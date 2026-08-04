`timescale 1ns/1ps

// Test inputs: outputs from end of wr_arbiter pipeline
// Test outputs: internal signals and outputs from memory pool response

// Proper bank allocation
// Proper memory pool access (no forwarding)
// Proper fowarding

module tb_memory_bank;

    localparam int DATA_W = 64;
    localparam int ADDR_W = 64;
    localparam int RELEASE_SET_DEPTH = 16;
    localparam logic [1:0] CMD_TX_COMMIT = 2'b10;
    localparam logic [1:0] CMD_LOAD = 2'b00;
    localparam int NUM_BANKS = 4;
    localparam int NUM_NODES = 2;
    // Response lane count: one memory lane + one forward lane per bank.
    localparam int NRESP = 2 * NUM_BANKS;

    localparam int BUF_DEPTH = 16;
    localparam int MEM_DEPTH = 1024;
    localparam int MEM_LATENCY = 10;

    localparam int CLK_PERIOD = 10;

    logic clk;
    logic rst;

    // ================ Bank Router ================

    // input from arbiter to br
    logic arb_valid;
    logic arb_we;
    logic [ADDR_W-1:0] arb_addr;
    logic [DATA_W-1:0] arb_wdata;
    logic [NUM_NODES-1:0] arb_worker;
    logic [1:0] arb_req_type;

    // output from br to arbiter
    logic br_ready;

    // output from br to banks
    logic [NUM_BANKS-1:0] br_valid;
    logic br_we;
    logic [ADDR_W-1:0] br_addr;
    logic [DATA_W-1:0] br_wdata;
    logic [NUM_NODES-1:0] br_worker;
    logic [1:0] br_req_type;

    // Input from memory pool (per bank) to br
    logic [NUM_BANKS-1:0] bank_ready;

    // combinational
    bank_router br(
        .in_valid_i(arb_valid),
        .in_we_i(arb_we),
        .in_addr_i(arb_addr),
        .in_wdata_i(arb_wdata),
        .in_worker_i(arb_worker),
        .in_req_type_i(arb_req_type),
        .in_ready_o(br_ready), // tells the arbiter whether it is ready

        // I/o with the NUM_BANKS wr_buf instances
        .bank_valid_o(br_valid), // one-hot on the selected bank
        .bank_we_o(br_we), // broadcast
        .bank_addr_o(br_addr), // broadcast
        .bank_wdata_o(br_wdata), // broadcast
        .bank_worker_o(br_worker), // broadcast
        .bank_req_type_o(br_req_type), // broadcast
        .bank_ready_i(bank_ready) // per-bank push_ready_o
    );

    // ================ Memory Banks (Buffers and pools) ================

    // ---- All response lanes -> resp_merge ----
    // lanes [0 .. NUM_BANKS-1] = memory-read responses (wr_buf)
    // lanes [NUM_BANKS .. 2*NUM_BANKS-1]= forwarded responses (load_fwd)
    logic [NRESP-1:0] r_valid;
    logic [NRESP-1:0][NUM_NODES-1:0] r_worker;
    logic [NRESP-1:0][DATA_W-1:0] r_rdata;

    genvar g;
    generate
        for (g = 0; g < NUM_BANKS; g++) begin : g_bank
            // wr_buf -> mem_pool
            logic pop_valid, pop_we;
            logic [ADDR_W-1:0] pop_addr;
            logic [DATA_W-1:0] pop_wdata;
            logic [NUM_NODES-1:0] pop_worker;
            logic [1:0] pop_req_type;
            logic mem_ready;
            logic [DATA_W-1:0] mem_rdata;

            wr_buf #(
                .NUM_NODES(NUM_NODES),
                .DATA_W(DATA_W),
                .ADDR_W(ADDR_W),
                .DEPTH(BUF_DEPTH)
            ) buf_inst (
                .clk_i(clk),
                .rst_i(rst),

                // Push side <- router (forwarding decided internally)
                .push_valid_i(br_valid[g]),
                .push_we_i(br_we),
                .push_addr_i(br_addr),
                .push_wdata_i(br_wdata),
                .push_worker_i(br_worker),
                .push_req_type_i(br_req_type),
                .push_ready_o(bank_ready[g]),

                // pop side
                .pop_valid_o(pop_valid),
                .pop_we_o(pop_we),
                .pop_addr_o(pop_addr),
                .pop_wdata_o(pop_wdata),
                .pop_worker_o(pop_worker),
                .pop_req_type_o(pop_req_type),

                .mem_ready_i(mem_ready),
                .mem_rdata_i(mem_rdata),

                // Memory-read response -> resp_merge (memory lane)
                .resp_valid_o (r_valid [g]),
                .resp_worker_o(r_worker[g]),
                .resp_rdata_o (r_rdata [g]),
                // Forwarded response -> resp_merge (forward lane)
                .fwd_valid_o (r_valid [NUM_BANKS + g]),
                .fwd_worker_o(r_worker[NUM_BANKS + g]),
                .fwd_rdata_o (r_rdata [NUM_BANKS + g])
            );

            mem_pool_stub #(
                .DATA_W(DATA_W),
                .ADDR_W(ADDR_W),
                .MEM_DEPTH(MEM_DEPTH),
                .MEM_LATENCY(MEM_LATENCY)
            ) mem (
                .clk_i(clk),
                .rst_i(rst),
                .mem_req_valid_i(pop_valid),
                .mem_we_i(pop_we),
                .mem_addr_i(pop_addr),
                .mem_wdata_i(pop_wdata),
                .mem_rdata_o(mem_rdata),
                .mem_rvalid_o(mem_ready)
            );
        end
    endgenerate


    // ================ Response Merger ================
    logic buf_resp_valid;
    logic [NUM_NODES-1:0] buf_resp_worker;
    logic [DATA_W-1:0] buf_resp_rdata;

    resp_merge rm(
        .clk_i(clk),
        .rst_i(rst),

        // From the NUM_BANKS wr_buf instances
        .in_valid_i(r_valid),
        .in_worker_i(r_worker),
        .in_rdata_i(r_rdata),

        // Consolidated ports to the nodes (single port)
        .resp_valid_o(buf_resp_valid),
        .resp_worker_o(buf_resp_worker),
        .resp_rdata_o(buf_resp_rdata)
    );


    // Generate clock
    initial clk = 0;
    always#(CLK_PERIOD/2) clk = ~clk;


    // ================ Helper Functions ================

    // Reset
    task automatic do_reset();
        rst = 1;
        repeat(2) @(posedge clk); #1; // slight skew so we're sampling after the edge
        rst = 0;
        @(posedge clk); #1;
    endtask

    // Push something into the br
    task automatic br_push(
        input logic arb_valid_i,
        input logic arb_we_i,
        input logic [ADDR_W-1:0] arb_addr_i,
        input logic [DATA_W-1:0] arb_wdata_i,
        input logic [NUM_NODES-1:0] arb_worker_i,
        input logic [1:0] arb_req_type_i
    );
        // pulse
        arb_valid = arb_valid_i;
        arb_we = arb_we_i;
        arb_addr = arb_addr_i;
        arb_wdata = arb_wdata_i;
        arb_worker = arb_worker_i;
        arb_req_type = arb_req_type_i;

        @(posedge clk); #1;

        arb_valid = '0;
        arb_we = '0;
        arb_addr = '0;
        arb_wdata = '0;
        arb_worker = '0;
        arb_req_type = '0;

        // bank availability logic is handled within the modules
        // will naturally push when the bank is available

    endtask



    // ================ Main Test Sequence ================
    initial begin
        
        logic [ADDR_W-1:0] t_addr [20];
        logic [DATA_W-1:0] t_data [20];
        logic [NUM_NODES-1:0] t_worker [20];
        logic [1:0] t_req_type [20];

        // stuff in 6 unrelated writes to addr 0000 first to keep buffer busy
        for(int i = 0; i < 6; i ++) begin
            t_addr[i] = 64'h0000_0000_0000_0000;
            t_data[i] = 64'h0000_0000_0000_0100;
            t_worker[i] = 1'b0;
            t_req_type[i] = 2'b10;
        end

        // 5 writes to addr 0001
        for(int i = 6; i < 11; i ++) begin
            t_addr[i] = 64'h1000_0000_0000_0000;
            t_data[i] = 64'h0000_0000_0000_0100 + i;
            t_worker[i] = 1'b0;
            t_req_type[i] = 2'b10;
        end

        // reads addr 0001
        for(int i = 11; i < 16; i ++) begin
            t_addr[i] = 64'h1000_0000_0000_0000; // read from same addresses
            t_data[i] = 64'h0000_0000_0000_0100 + i - 5; // irrelevant
            t_worker[i] = 1'b0;
            t_req_type[i] = 2'b0;
        end

        do_reset();
        
        // waveform dump
        $dumpfile("tb_memory_bank.vcd");
        $dumpvars(0, tb_memory_bank);

        $display("Load forwarding test");
        for(int i = 0; i < 16; i ++) begin
            if(t_req_type[i] == 2'b10) // commit
                br_push(1, 1, t_addr[i], t_data[i], t_worker[i], t_req_type[i]);
            else
                br_push(1, 0, t_addr[i], t_data[i], t_worker[i], t_req_type[i]);
        end
        
        repeat(50) @(posedge clk); #1;

        $finish;
    end


endmodule
