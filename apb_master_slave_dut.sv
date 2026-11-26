// =============================================================================
// APB TOP WRAPPER
// -----------------------------------------------------------------------------
// Connects the APB master to the APB slave (memory).
// The testbench drives only the request side (transfer / wr_rd / addr / wdata)
// and observes rdata. The APB bus itself is internal to this wrapper.
// =============================================================================
module apb_top_wrapper #(
    parameter int DATA_WIDTH = 32,
    parameter int DEPTH      = 8,
    parameter int ADDR_WIDTH = $clog2(DEPTH)
) (
    // Clock and reset
    input  logic                  pclk_i,
    input  logic                  prst_n,

    // Request side (from TB / upstream bus)
    input  logic                  transfer,
    input  logic                  wr_rd,      // 1 = write, 0 = read
    input  logic [ADDR_WIDTH-1:0] addr,
    input  logic [DATA_WIDTH-1:0] wdata,
    output logic [DATA_WIDTH-1:0] rdata
);

    // -------------------------------------------------------------------------
    // Internal APB bus (master <-> slave)
    // -------------------------------------------------------------------------
    logic                  psel;
    logic                  penable;
    logic [ADDR_WIDTH-1:0] paddr;
    logic                  pwrite;
    logic [DATA_WIDTH-1:0] pwdata;
    logic                  pready;
    logic [DATA_WIDTH-1:0] prdata;

    // -------------------------------------------------------------------------
    // APB master
    // -------------------------------------------------------------------------
    apb_master #(
        .DATA_WIDTH (DATA_WIDTH),
        .DEPTH      (DEPTH),
        .ADDR_WIDTH (ADDR_WIDTH)
    ) DUT_APB_MASTER (
        .pclk_i   (pclk_i),
        .prst_n   (prst_n),
        .transfer (transfer),
        .wr_rd    (wr_rd),
        .addr     (addr),
        .wdata    (wdata),
        .rdata    (rdata),
        .psel     (psel),
        .penable  (penable),
        .paddr    (paddr),
        .pwrite   (pwrite),
        .pwdata   (pwdata),
        .pready   (pready),
        .prdata   (prdata)
    );

    // -------------------------------------------------------------------------
    // APB slave (memory)
    // -------------------------------------------------------------------------
    apb_slave #(
        .DATA_WIDTH (DATA_WIDTH),
        .DEPTH      (DEPTH),
        .ADDR_WIDTH (ADDR_WIDTH)
    ) DUT_APB_SLAVE (
        .pclk_i   (pclk_i),
        .prst_n   (prst_n),
        .psel     (psel),
        .penable  (penable),
        .paddr    (paddr),
        .pwrite   (pwrite),
        .pwdata   (pwdata),
        .pready   (pready),
        .prdata   (prdata)
    );

endmodule


// =============================================================================
// APB MASTER
// -----------------------------------------------------------------------------
// FSM : IDLE -> SETUP -> ACCESS -> (IDLE | SETUP for back-to-back)
//
//   IDLE   : psel = 0, penable = 0. Waits for transfer.
//   SETUP  : psel = 1, penable = 0. Lasts exactly one cycle.
//   ACCESS : psel = 1, penable = 1. Held until pready = 1.
//
// paddr / pwdata / pwrite are registered on the edge that enters SETUP and
// stay stable through ACCESS, as the APB spec requires.
// =============================================================================
module apb_master #(
    parameter int DATA_WIDTH = 32,
    parameter int DEPTH      = 8,
    parameter int ADDR_WIDTH = $clog2(DEPTH)
) (
    // Clock and reset
    input  logic                  pclk_i,
    input  logic                  prst_n,

    // Request side (from TB / upstream bus)
    input  logic                  transfer,
    input  logic                  wr_rd,      // 1 = write, 0 = read
    input  logic [ADDR_WIDTH-1:0] addr,
    input  logic [DATA_WIDTH-1:0] wdata,
    output logic [DATA_WIDTH-1:0] rdata,

    // APB side (to slave)
    output logic                  psel,
    output logic                  penable,
    output logic [ADDR_WIDTH-1:0] paddr,
    output logic                  pwrite,
    output logic [DATA_WIDTH-1:0] pwdata,
    input  logic                  pready,
    input  logic [DATA_WIDTH-1:0] prdata
);

    // -------------------------------------------------------------------------
    // State encoding
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        IDLE   = 2'b00,
        SETUP  = 2'b01,
        ACCESS = 2'b10
    } state_t;

    state_t cs, ns;

    // -------------------------------------------------------------------------
    // Next-state logic (combinational)
    // -------------------------------------------------------------------------
    always_comb begin
        ns = cs;                                  // default: hold state

        case (cs)
            IDLE: begin
                if (transfer)
                    ns = SETUP;                   // new request
            end

            SETUP: begin
                ns = ACCESS;                      // SETUP is always one cycle
            end

            ACCESS: begin
                if (pready) begin                 // transfer completes this edge
                    if (transfer)
                        ns = SETUP;               // back-to-back request
                    else
                        ns = IDLE;
                end
                // pready = 0 : wait state, stay in ACCESS
            end

            default: begin
                ns = IDLE;                        // recover from illegal state
            end
        endcase
    end

    // -------------------------------------------------------------------------
    // Sequential logic: state register, APB data registers, read capture
    // -------------------------------------------------------------------------
    always_ff @(posedge pclk_i or negedge prst_n) begin
        if (!prst_n) begin
            cs     <= IDLE;
            paddr  <= '0;
            pwdata <= '0;
            pwrite <= 1'b0;
            rdata  <= '0;
        end
        else begin
            cs <= ns;

            // Latch request when entering SETUP; hold through ACCESS
            if (ns == SETUP) begin
                paddr  <= addr;
                pwdata <= wdata;
                pwrite <= wr_rd;
            end

            // Capture read data on the edge where a read completes
            if ((cs == ACCESS) && pready && !pwrite) begin
                rdata <= prdata;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Output decode (Moore outputs from current state)
    // -------------------------------------------------------------------------
    always_comb begin
        psel    = 1'b0;                           // defaults (IDLE)
        penable = 1'b0;

        case (cs)
            SETUP: begin
                psel    = 1'b1;
                penable = 1'b0;
            end

            ACCESS: begin
                psel    = 1'b1;
                penable = 1'b1;
            end

            default: begin
                psel    = 1'b0;
                penable = 1'b0;
            end
        endcase
    end

endmodule


// =============================================================================
// APB SLAVE (MEMORY)
// -----------------------------------------------------------------------------
// DEPTH x DATA_WIDTH memory with an APB interface. Zero wait states.
//
//   pready : registered; low in reset, high from the first edge after reset.
//   Write  : apb_mem[paddr] <= pwdata on the edge that completes ACCESS
//            (psel && penable && pready && pwrite).
//   Read   : registered. prdata is loaded on the edge that ends SETUP
//            (psel && !penable && !pwrite), so it is stable for the whole
//            ACCESS phase and the master captures it on the completing edge.
// =============================================================================
module apb_slave #(
    parameter int DATA_WIDTH = 32,
    parameter int DEPTH      = 8,
    parameter int ADDR_WIDTH = $clog2(DEPTH)
) (
    // Clock and reset
    input  logic                  pclk_i,
    input  logic                  prst_n,

    // APB side (from master)
    input  logic                  psel,
    input  logic                  penable,
    input  logic [ADDR_WIDTH-1:0] paddr,
    input  logic                  pwrite,
    input  logic [DATA_WIDTH-1:0] pwdata,
    output logic                  pready,
    output logic [DATA_WIDTH-1:0] prdata
);

    // -------------------------------------------------------------------------
    // Storage
    // -------------------------------------------------------------------------
    logic [DATA_WIDTH-1:0] apb_mem [DEPTH];

    // -------------------------------------------------------------------------
    // Ready + write path
    // -------------------------------------------------------------------------
    always_ff @(posedge pclk_i or negedge prst_n) begin
        if (!prst_n) begin
            pready <= 1'b0;

            for (int i = 0; i < DEPTH; i++) begin
                apb_mem[i] <= '0;             // clear memory on reset
            end
        end
        else begin
            pready <= 1'b1;                   // zero wait states

            // Write on the edge where the transfer completes
            if (psel && penable && pready && pwrite) begin
                apb_mem[paddr] <= pwdata;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Read path (registered)
    // SETUP of a read is the only cycle with psel = 1, penable = 0, pwrite = 0.
    // -------------------------------------------------------------------------
    always_ff @(posedge pclk_i or negedge prst_n) begin
        if (!prst_n) begin
            prdata <= '0;
        end
        else if (psel && !penable && !pwrite) begin
            prdata <= apb_mem[paddr];
        end
    end

endmodule


// =============================================================================
// TESTBENCH
// -----------------------------------------------------------------------------
// Race-free driving rule used throughout:
//   * Wait for the clock edge, then drive DUT inputs with NON-BLOCKING (<=).
//   * The DUT's always_ff blocks triggered by the same edge therefore always
//     sample the OLD values; the new values are seen at the NEXT edge.
//   * This makes behaviour deterministic on every simulator, independent of
//     the order in which the TB thread and the DUT processes are scheduled.
//
// Sampling rule:
//   * Outputs (rdata) are read at the negedge, half a cycle after they
//     update, so the TB never samples on the same edge the DUT changes them.
// =============================================================================
module tb_apb;

    parameter int DATA_WIDTH = 32;
    parameter int DEPTH      = 8;
    parameter int ADDR_WIDTH = $clog2(DEPTH);

    // -------------------------------------------------------------------------
    // Clock and reset
    // -------------------------------------------------------------------------
    logic pclk_i;
    logic prst_n;

    // -------------------------------------------------------------------------
    // Request side signals (TB -> DUT) and read data (DUT -> TB)
    // -------------------------------------------------------------------------
    logic                  transfer;
    logic                  wr_rd;
    logic [ADDR_WIDTH-1:0] addr;
    logic [DATA_WIDTH-1:0] wdata;
    logic [DATA_WIDTH-1:0] rdata;

    // -------------------------------------------------------------------------
    // DUT instance
    // -------------------------------------------------------------------------
    apb_top_wrapper #(
        .DATA_WIDTH (DATA_WIDTH),
        .DEPTH      (DEPTH),
        .ADDR_WIDTH (ADDR_WIDTH)
    ) DUT_WRAPPER (.*);

    // -------------------------------------------------------------------------
    // Clock generation: 10 time-unit period, first posedge at t = 5
    // -------------------------------------------------------------------------
    initial pclk_i = 1'b0;
    always #5 pclk_i = ~pclk_i;

    // -------------------------------------------------------------------------
    // Reset task
    // Asserted at t = 0 (before any clock edge), released on a negedge so the
    // release never coincides with a posedge where the DUT samples.
    // -------------------------------------------------------------------------
    task automatic rst_dut();
        prst_n   = 1'b0;
        transfer = 1'b0;
        wr_rd    = 1'b0;
        addr     = '0;
        wdata    = '0;

        repeat (2) @(negedge pclk_i);
        prst_n = 1'b1;
    endtask

    // -------------------------------------------------------------------------
    // Write task
    // Timeline (E = posedge where the task starts driving):
    //   E   : TB raises transfer (DUT still sees transfer = 0)
    //   E+1 : DUT sees transfer = 1 -> enters SETUP, latches addr/wdata;
    //         TB drops transfer
    //   E+2 : ACCESS
    //   E+3 : write completes (pready = 1, no wait states)
    // addr / wdata / wr_rd are left unchanged after the task returns.
    // -------------------------------------------------------------------------
    task automatic write_mem(
        input logic [ADDR_WIDTH-1:0] mem_addr,
        input logic [DATA_WIDTH-1:0] mem_data
    );
        @(posedge pclk_i);
        transfer <= 1'b1;
        wr_rd    <= 1'b1;
        addr     <= mem_addr;
        wdata    <= mem_data;

        @(posedge pclk_i);
        transfer <= 1'b0;

        $display("t=%0t  WRITE addr=%0d  wdata=%0h", $time, mem_addr, mem_data);
    endtask

    // -------------------------------------------------------------------------
    // Read task (assumes no wait states: pready = 1)
    // Timeline (E = posedge where the task starts driving):
    //   E   : TB raises transfer
    //   E+1 : DUT enters SETUP, latches addr; TB drops transfer
    //   E+2 : ACCESS
    //   E+3 : read completes, rdata <= prdata
    //   negedge after E+3 : TB samples rdata
    // -------------------------------------------------------------------------
    task automatic read_mem(
        input logic [ADDR_WIDTH-1:0] mem_addr
    );
        @(posedge pclk_i);
        transfer <= 1'b1;
        wr_rd    <= 1'b0;
        addr     <= mem_addr;

        @(posedge pclk_i);
        transfer <= 1'b0;

        repeat (2) @(posedge pclk_i);             // E+2 (ACCESS), E+3 (complete)
        @(negedge pclk_i);                        // sample away from the edge

        $display("t=%0t  READ  addr=%0d  rdata=%0h", $time, mem_addr, rdata);
    endtask

    // -------------------------------------------------------------------------
    // Stimulus
    // -------------------------------------------------------------------------
    initial begin
        rst_dut();

        write_mem(3'h6, 32'hdead_beef);
        read_mem (3'h6);

        write_mem(3'h3, 32'hcead_deef);
        read_mem (3'h3);

        #200;
        $writememh("mem_dump.txt", DUT_WRAPPER.DUT_APB_SLAVE.apb_mem);
        $finish;
    end

    // -------------------------------------------------------------------------
    // Waveform dump
    // -------------------------------------------------------------------------
    initial begin
        $dumpfile("wave_1.vcd");
        $dumpvars(0, tb_apb);
    end

endmodule
