// =============================================================
// APB TOP WRAPPER
// Connects APB master and APB slave; TB drives request side only
// =============================================================
module apb_top_wrapper(pclk_i, prst_n, transfer, wr_rd, addr, wdata, rdata);

    parameter DATA_WIDTH = 32;
    parameter DEPTH      = 8;
    parameter ADDR_WIDTH = $clog2(DEPTH);

    // ---------------------------------------------
    // Clock and reset
    // ---------------------------------------------
    input logic pclk_i, prst_n;

    // ---------------------------------------------
    // Request side (AHB side / TB)
    // ---------------------------------------------
    input  logic                  transfer;
    input  logic                  wr_rd;
    input  logic [ADDR_WIDTH-1:0] addr;
    input  logic [DATA_WIDTH-1:0] wdata;
    output logic [DATA_WIDTH-1:0] rdata;

    // ---------------------------------------------
    // Internal APB bus (master <-> slave)
    // ---------------------------------------------
    logic                  psel;
    logic                  penable;
    logic [ADDR_WIDTH-1:0] paddr;
    logic                  pwrite;
    logic [DATA_WIDTH-1:0] pwdata;
    logic                  pready;
    logic [DATA_WIDTH-1:0] prdata;

    // ---------------------------------------------
    // Master instantiation
    // ---------------------------------------------
    apb_master #(DATA_WIDTH, DEPTH, ADDR_WIDTH) DUT_APB_MASTER (
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

    // ---------------------------------------------
    // Slave instantiation
    // ---------------------------------------------
    apb_slave #(DATA_WIDTH, DEPTH, ADDR_WIDTH) DUT_APB_SLAVE (
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


// =============================================================
// APB MASTER
// FSM: IDLE -> SETUP -> ACCESS
// =============================================================
module apb_master(pclk_i, prst_n, psel, penable, paddr, pwrite, pwdata, pready, prdata,
                  transfer, wdata, addr, wr_rd, rdata);

    parameter DATA_WIDTH = 32;
    parameter DEPTH      = 8;
    parameter ADDR_WIDTH = $clog2(DEPTH);

    // ---------------------------------------------
    // Clock and reset
    // ---------------------------------------------
    input pclk_i, prst_n;

    // ---------------------------------------------
    // Request side (AHB side)
    // ---------------------------------------------
    input  logic                  transfer;
    input  logic                  wr_rd;
    input  logic [ADDR_WIDTH-1:0] addr;
    input  logic [DATA_WIDTH-1:0] wdata;
    output logic [DATA_WIDTH-1:0] rdata;

    // ---------------------------------------------
    // APB side (to slave)
    // ---------------------------------------------
    output logic                  psel;
    output logic                  penable;
    output logic [ADDR_WIDTH-1:0] paddr;
    output logic                  pwrite;
    output logic [DATA_WIDTH-1:0] pwdata;
    input  logic                  pready;
    input  logic [DATA_WIDTH-1:0] prdata;

    // ---------------------------------------------
    // State encoding
    // ---------------------------------------------
    typedef enum logic [1:0] {
        IDLE,
        SETUP,
        ACCESS
    } state_t;

    state_t ns, cs;

    // ---------------------------------------------
    // Next-state logic (combinational)
    // ---------------------------------------------
    always_comb begin

        ns = cs;                              // default: hold state

        case (cs)

            IDLE: begin
                if (transfer) ns = SETUP;     // new request
            end

            SETUP: begin
                ns = ACCESS;                  // SETUP always lasts 1 cycle
            end

            ACCESS: begin
                if (pready) begin             // transfer completes
                    if (transfer) ns = SETUP; // back-to-back
                    else          ns = IDLE;
                end
            end

        endcase

    end

    // ---------------------------------------------
    // State register + data registers + read capture
    // ---------------------------------------------
    always_ff @(posedge pclk_i or negedge prst_n) begin

        if (!prst_n) begin
            cs <= IDLE;

            // Data signals
            paddr  <= '0;
            pwdata <= '0;
            pwrite <= 1'b0;
            rdata  <= '0;
        end

        else begin
            cs <= ns;

            // Data flops: load when entering SETUP, hold otherwise
            if (ns == SETUP) begin
                paddr  <= addr;
                pwdata <= wdata;
                pwrite <= wr_rd;
            end

            // Read data: capture at the edge where a read completes
            if (cs == ACCESS && pready && !pwrite) begin
                rdata <= prdata;
            end
        end

    end

    // ---------------------------------------------
    // Output logic (decoded from cs)
    // ---------------------------------------------
    always_comb begin

        psel    = 1'b0;                       // defaults
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


// =============================================================
// APB SLAVE
// Memory-based slave, zero wait states, registered read data
// =============================================================
module apb_slave(pclk_i, prst_n, psel, penable, paddr, pwrite, pwdata, pready, prdata);

    parameter DATA_WIDTH = 32;
    parameter DEPTH      = 8;
    parameter ADDR_WIDTH = $clog2(DEPTH);

    // ---------------------------------------------
    // Clock and reset
    // ---------------------------------------------
    input pclk_i, prst_n;

    // ---------------------------------------------
    // APB side (from master)
    // ---------------------------------------------
    input  logic                  psel;
    input  logic                  penable;
    input  logic [ADDR_WIDTH-1:0] paddr;
    input  logic                  pwrite;
    input  logic [DATA_WIDTH-1:0] pwdata;

    output logic                  pready;
    output logic [DATA_WIDTH-1:0] prdata;

    // ---------------------------------------------
    // Memory: DEPTH entries of DATA_WIDTH bits
    // ---------------------------------------------
    logic [DATA_WIDTH-1:0] apb_mem [DEPTH];

    // ---------------------------------------------
    // Ready + write logic
    // ---------------------------------------------
    always_ff @(posedge pclk_i or negedge prst_n) begin

        if (!prst_n) begin
            pready <= 1'b0;

            // Clear memory on reset
            for (int i = 0; i < DEPTH; i++) begin
                apb_mem[i] <= '0;
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

    /* Combinational read: can glitch when its inputs change in different delta cycles
    // ---------------------------------------------
    // Read logic: data valid during ACCESS
    // ---------------------------------------------
    assign prdata = (psel && penable && !pwrite) ? apb_mem[paddr] : '0;
    */

    // ---------------------------------------------
    // Read logic (registered): load during SETUP of a read,
    // so prdata is valid from the start of ACCESS
    // ---------------------------------------------
    always_ff @(posedge pclk_i or negedge prst_n) begin

        if (!prst_n)
            prdata <= '0;
        else if (psel && !penable && !pwrite) // SETUP of a read: how the slave sees SETUP
            prdata <= apb_mem[paddr];

    end

endmodule


// =============================================================
// APB TOP WRAPPER
// Connects APB master and APB slave; TB drives request side only
// =============================================================
module apb_top_wrapper(pclk_i, prst_n, transfer, wr_rd, addr, wdata, rdata);

    parameter DATA_WIDTH = 32;
    parameter DEPTH      = 8;
    parameter ADDR_WIDTH = $clog2(DEPTH);

    // ---------------------------------------------
    // Clock and reset
    // ---------------------------------------------
    input logic pclk_i, prst_n;

    // ---------------------------------------------
    // Request side (AHB side / TB)
    // ---------------------------------------------
    input  logic                  transfer;
    input  logic                  wr_rd;
    input  logic [ADDR_WIDTH-1:0] addr;
    input  logic [DATA_WIDTH-1:0] wdata;
    output logic [DATA_WIDTH-1:0] rdata;

    // ---------------------------------------------
    // Internal APB bus (master <-> slave)
    // ---------------------------------------------
    logic                  psel;
    logic                  penable;
    logic [ADDR_WIDTH-1:0] paddr;
    logic                  pwrite;
    logic [DATA_WIDTH-1:0] pwdata;
    logic                  pready;
    logic [DATA_WIDTH-1:0] prdata;

    // ---------------------------------------------
    // Master instantiation
    // ---------------------------------------------
    apb_master #(DATA_WIDTH, DEPTH, ADDR_WIDTH) DUT_APB_MASTER (
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

    // ---------------------------------------------
    // Slave instantiation
    // ---------------------------------------------
    apb_slave #(DATA_WIDTH, DEPTH, ADDR_WIDTH) DUT_APB_SLAVE (
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


// =============================================================
// APB MASTER
// FSM: IDLE -> SETUP -> ACCESS
// =============================================================
module apb_master(pclk_i, prst_n, psel, penable, paddr, pwrite, pwdata, pready, prdata,
                  transfer, wdata, addr, wr_rd, rdata);

    parameter DATA_WIDTH = 32;
    parameter DEPTH      = 8;
    parameter ADDR_WIDTH = $clog2(DEPTH);

    // ---------------------------------------------
    // Clock and reset
    // ---------------------------------------------
    input pclk_i, prst_n;

    // ---------------------------------------------
    // Request side (AHB side)
    // ---------------------------------------------
    input  logic                  transfer;
    input  logic                  wr_rd;
    input  logic [ADDR_WIDTH-1:0] addr;
    input  logic [DATA_WIDTH-1:0] wdata;
    output logic [DATA_WIDTH-1:0] rdata;

    // ---------------------------------------------
    // APB side (to slave)
    // ---------------------------------------------
    output logic                  psel;
    output logic                  penable;
    output logic [ADDR_WIDTH-1:0] paddr;
    output logic                  pwrite;
    output logic [DATA_WIDTH-1:0] pwdata;
    input  logic                  pready;
    input  logic [DATA_WIDTH-1:0] prdata;

    // ---------------------------------------------
    // State encoding
    // ---------------------------------------------
    typedef enum logic [1:0] {
        IDLE,
        SETUP,
        ACCESS
    } state_t;

    state_t ns, cs;

    // ---------------------------------------------
    // Next-state logic (combinational)
    // ---------------------------------------------
    always_comb begin

        ns = cs;                              // default: hold state

        case (cs)

            IDLE: begin
                if (transfer) ns = SETUP;     // new request
            end

            SETUP: begin
                ns = ACCESS;                  // SETUP always lasts 1 cycle
            end

            ACCESS: begin
                if (pready) begin             // transfer completes
                    if (transfer) ns = SETUP; // back-to-back
                    else          ns = IDLE;
                end
            end

        endcase

    end

    // ---------------------------------------------
    // State register + data registers + read capture
    // ---------------------------------------------
    always_ff @(posedge pclk_i or negedge prst_n) begin

        if (!prst_n) begin
            cs <= IDLE;

            // Data signals
            paddr  <= '0;
            pwdata <= '0;
            pwrite <= 1'b0;
            rdata  <= '0;
        end

        else begin
            cs <= ns;

            // Data flops: load when entering SETUP, hold otherwise
            if (ns == SETUP) begin
                paddr  <= addr;
                pwdata <= wdata;
                pwrite <= wr_rd;
            end

            // Read data: capture at the edge where a read completes
            if (cs == ACCESS && pready && !pwrite) begin
                rdata <= prdata;
            end
        end

    end

    // ---------------------------------------------
    // Output logic (decoded from cs)
    // ---------------------------------------------
    always_comb begin

        psel    = 1'b0;                       // defaults
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


// =============================================================
// APB SLAVE
// Memory-based slave, zero wait states, registered read data
// =============================================================
module apb_slave(pclk_i, prst_n, psel, penable, paddr, pwrite, pwdata, pready, prdata);

    parameter DATA_WIDTH = 32;
    parameter DEPTH      = 8;
    parameter ADDR_WIDTH = $clog2(DEPTH);

    // ---------------------------------------------
    // Clock and reset
    // ---------------------------------------------
    input pclk_i, prst_n;

    // ---------------------------------------------
    // APB side (from master)
    // ---------------------------------------------
    input  logic                  psel;
    input  logic                  penable;
    input  logic [ADDR_WIDTH-1:0] paddr;
    input  logic                  pwrite;
    input  logic [DATA_WIDTH-1:0] pwdata;

    output logic                  pready;
    output logic [DATA_WIDTH-1:0] prdata;

    // ---------------------------------------------
    // Memory: DEPTH entries of DATA_WIDTH bits
    // ---------------------------------------------
    logic [DATA_WIDTH-1:0] apb_mem [DEPTH];

    // ---------------------------------------------
    // Ready + write logic
    // ---------------------------------------------
    always_ff @(posedge pclk_i or negedge prst_n) begin

        if (!prst_n) begin
            pready <= 1'b0;

            // Clear memory on reset
            for (int i = 0; i < DEPTH; i++) begin
                apb_mem[i] <= '0;
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

    /* Combinational read: can glitch when its inputs change in different delta cycles
    // ---------------------------------------------
    // Read logic: data valid during ACCESS
    // ---------------------------------------------
    assign prdata = (psel && penable && !pwrite) ? apb_mem[paddr] : '0;
    */

    // ---------------------------------------------
    // Read logic (registered): load during SETUP of a read,
    // so prdata is valid from the start of ACCESS
    // ---------------------------------------------
    always_ff @(posedge pclk_i or negedge prst_n) begin

        if (!prst_n)
            prdata <= '0;
        else if (psel && !penable && !pwrite) // SETUP of a read: how the slave sees SETUP
            prdata <= apb_mem[paddr];

    end

endmodule


// =============================================================
// TESTBENCH
// =============================================================
module tb_apb;

    parameter DATA_WIDTH = 32;
    parameter DEPTH      = 8;
    parameter ADDR_WIDTH = $clog2(DEPTH);

    // ---------------------------------------------
    // Clock and reset
    // ---------------------------------------------
    logic pclk_i, prst_n;

    // ---------------------------------------------
    // APB wrapper side (request inputs / read data)
    // ---------------------------------------------
    logic                  transfer;
    logic                  wr_rd;
    logic [ADDR_WIDTH-1:0] addr;
    logic [DATA_WIDTH-1:0] wdata;
    logic [DATA_WIDTH-1:0] rdata;

    // ---------------------------------------------
    // DUT instance
    // ---------------------------------------------
    apb_top_wrapper DUT_WRAPPER (.*);

    // ---------------------------------------------
    // Clock generation
    // ---------------------------------------------
    initial pclk_i = 1'b0;
    always #5 pclk_i = ~pclk_i;

    // ---------------------------------------------
    // Reset task
    // ---------------------------------------------
    task rst_dut();
        prst_n = 1'b0;

        // Inputs to DUT from TB = zero
        transfer = '0;
        wr_rd    = '0;
        addr     = '0;
        wdata    = '0;

        repeat (2) @(negedge pclk_i);
        prst_n = 1'b1;
    endtask

    // ---------------------------------------------
    // Write task (drive at posedge with blocking =)
    // ---------------------------------------------
    task write_mem(logic [ADDR_WIDTH-1:0] mem_addr, logic [DATA_WIDTH-1:0] mem_data);

        @(posedge pclk_i);
        transfer = 1'b1;
        addr     = mem_addr;
        wr_rd    = 1'b1;
        wdata    = mem_data;

        @(posedge pclk_i);
        transfer = 1'b0;

    endtask

    // ---------------------------------------------
    // Read task (drive at posedge with blocking =, no wait states assumed)
    // ---------------------------------------------
    task read_mem(logic [ADDR_WIDTH-1:0] mem_addr);

        @(posedge pclk_i);
        transfer = 1'b1;
        wr_rd    = 1'b0;
        addr     = mem_addr;

        @(posedge pclk_i);
        transfer = 1'b0;

        repeat (2) @(posedge pclk_i);         // SETUP -> ACCESS, then read completes
        @(negedge pclk_i);                    // let rdata settle
        $display("t=%0t  READ  addr=%0d  rdata=%0h", $time, mem_addr, rdata);

    endtask

    // ---------------------------------------------
    // Stimulus
    // ---------------------------------------------
    initial begin
        rst_dut();

        write_mem(3'h6, 32'hdead_beef);
        
        //@(posedge pclk_i);
        read_mem(3'h6);

        write_mem(3'h3, 32'hcead_deef);
        //@(posedge pclk_i);
        read_mem(3'h3);
        

        #200;
        $writememh("mem_dump.txt", DUT_WRAPPER.DUT_APB_SLAVE.apb_mem);
        $finish();
    end

    // ---------------------------------------------
    // Waveform dump
    // ---------------------------------------------
    initial begin
        $dumpfile("wave_1.vcd");
        $dumpvars(0, tb_apb);
    end

endmodule
