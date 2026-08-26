module top #(
    parameter int DATA_W = 64,
    parameter int ADDR_W = 64,
    parameter int NUM_NODES = 4,
    parameter int RELEASE_SET_DEPTH = 16,
    parameter int CXL_TABLE_DEPTH = 32,
    parameter int BUF_DEPTH = 16,   // per-bank wr_buf FIFO depth (also SQ depth)
    parameter int MEM_DEPTH = 1024, // per-bank stub memory size (words)
    parameter int MEM_LATENCY = 1,  // stub read latency in cycles (>=1)
    parameter int NUM_BANKS = 4     // number of interleaved memory banks
)(
    input logic clk_i,
    input logic rst_i,

    input logic [NUM_NODES-1:0] req_valid_i,
    input logic [1:0] tx_signal_i,
    input logic [ADDR_W-1:0] load_addr_i,
    input logic [RELEASE_SET_DEPTH-1:0] release_is_write_i,
    input logic [RELEASE_SET_DEPTH-1:0] release_is_read_i,
    input logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr_i,
    input logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data_i,

    output logic [NUM_NODES-1:0] req_ready_o,

    // Controller-driven completion (TX_ABORT only)
    output logic [NUM_NODES-1:0] ctrl_resp_valid_o,
    output logic [1:0] ctrl_comp_signal_o,

    // Merged LOAD completion (forwarded + memory-read paths)
    output logic buf_resp_valid_o,
    output logic [NUM_NODES-1:0] buf_resp_worker_o,
    output logic [DATA_W-1:0] buf_resp_rdata_o
);

    // Response lane count: one memory lane + one forward lane per bank.
    localparam int NRESP = 2 * NUM_BANKS;

    // ---- Controller -> Arbiter ----
    logic ctrl_req_valid;
    logic [1:0] ctrl_req_type;
    logic [ADDR_W-1:0] ctrl_addr;
    logic [RELEASE_SET_DEPTH-1:0] ctrl_rel_is_write;
    logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] ctrl_rel_addr;
    logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] ctrl_rel_data;
    logic [NUM_NODES-1:0] ctrl_worker;

    // ---- Arbiter -> Controller ----
    logic arb_busy;
    logic arb_done;

    // ---- Arbiter -> Router (single beat stream) ----
    logic push_valid;
    logic push_we;
    logic [ADDR_W-1:0] push_addr;
    logic [DATA_W-1:0] push_wdata;
    logic [NUM_NODES-1:0] push_worker;
    logic [1:0] push_req_type;
    logic router_ready;

    // ---- Router -> banks (broadcast payload + one-hot valid) ----
    logic [NUM_BANKS-1:0] bank_valid;
    logic                 bank_we;
    logic [ADDR_W-1:0]    bank_addr;
    logic [DATA_W-1:0]    bank_wdata;
    logic [NUM_NODES-1:0] bank_worker;
    logic [1:0]           bank_req_type;
    logic [NUM_BANKS-1:0] bank_ready;   // = per-bank load_fwd in_ready

    // ---- All response lanes -> resp_merge ----
    // lanes [0 .. NUM_BANKS-1]          = memory-read responses (wr_buf)
    // lanes [NUM_BANKS .. 2*NUM_BANKS-1]= forwarded responses (load_fwd)
    logic [NRESP-1:0] r_valid;
    logic [NRESP-1:0][NUM_NODES-1:0] r_worker;
    logic [NRESP-1:0][DATA_W-1:0] r_rdata;

    // ================ Controller ================
    cxl_controller #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .NUM_NODES(NUM_NODES),
        .RELEASE_SET_DEPTH(RELEASE_SET_DEPTH),
        .CXL_TABLE_DEPTH(CXL_TABLE_DEPTH)
    ) dut (
        .clk_i(clk_i),
        .rst_i(rst_i),

        .req_valid_i(req_valid_i),
        .tx_signal_i(tx_signal_i),
        .load_addr_i(load_addr_i),
        .release_is_write_i(release_is_write_i),
        .release_is_read_i(release_is_read_i),
        .release_addr_i(release_addr_i),
        .release_data_i(release_data_i),

        .buffer_full_i(arb_busy),

        .req_ready_o(req_ready_o),
        .resp_valid_o(ctrl_resp_valid_o),
        .comp_signal_o(ctrl_comp_signal_o),

        .mem_req_valid_o(ctrl_req_valid),
        .mem_we_o(),
        .mem_addr_o(ctrl_addr),
        .release_is_write_o(ctrl_rel_is_write),
        .release_addr_o(ctrl_rel_addr),
        .release_data_o(ctrl_rel_data),
        .mem_worker_o(ctrl_worker),
        .mem_req_type_o(ctrl_req_type)
    );

    // ================ Write Arbiter (serializer) ================
    wr_arbiter #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .NUM_NODES(NUM_NODES),
        .RELEASE_SET_DEPTH(RELEASE_SET_DEPTH),
        .DEPTH(BUF_DEPTH)
    ) arb_inst (
        .clk_i(clk_i),
        .rst_i(rst_i),

        .req_valid_i(ctrl_req_valid),
        .req_type_i(ctrl_req_type),
        .load_addr_i(ctrl_addr),
        .release_addr_i(ctrl_rel_addr),
        .release_is_write_i(ctrl_rel_is_write),
        .release_data_i(ctrl_rel_data),
        .worker_i(ctrl_worker),

        .busy_o(arb_busy),
        .done_o(arb_done),

        .push_ready_i(router_ready),

        .push_valid_o(push_valid),
        .push_we_o(push_we),
        .push_addr_o(push_addr),
        .push_wdata_o(push_wdata),
        .push_worker_o(push_worker),
        .push_req_type_o(push_req_type)
    );

    // ================ Bank Router (comb 1->N demux) ================
    bank_router #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .NUM_NODES(NUM_NODES),
        .NUM_BANKS(NUM_BANKS)
    ) router_inst (
        .in_valid_i(push_valid),
        .in_we_i(push_we),
        .in_addr_i(push_addr),
        .in_wdata_i(push_wdata),
        .in_worker_i(push_worker),
        .in_req_type_i(push_req_type),
        .in_ready_o(router_ready),

        .bank_valid_o(bank_valid),
        .bank_we_o(bank_we),
        .bank_addr_o(bank_addr),
        .bank_wdata_o(bank_wdata),
        .bank_worker_o(bank_worker),
        .bank_req_type_o(bank_req_type),
        .bank_ready_i(bank_ready)
    );

    // ================ Per-bank: wr_buf (with internal forwarding) + mem_pool ====
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
                .clk_i(clk_i),
                .rst_i(rst_i),

                // Push side <- router (forwarding decided internally)
                .push_valid_i(bank_valid[g]),
                .push_we_i(bank_we),
                .push_addr_i(bank_addr),
                .push_wdata_i(bank_wdata),
                .push_worker_i(bank_worker),
                .push_req_type_i(bank_req_type),
                .push_ready_o(bank_ready[g]),

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
                .clk_i(clk_i),
                .rst_i(rst_i),
                .mem_req_valid_i(pop_valid),
                .mem_we_i(pop_we),
                .mem_addr_i(pop_addr),
                .mem_wdata_i(pop_wdata),
                .mem_rdata_o(mem_rdata),
                .mem_rvalid_o(mem_ready)
            );
        end
    endgenerate

    // ================ Response Merge (2*NUM_BANKS lanes) ================
    resp_merge #(
        .DATA_W(DATA_W),
        .NUM_NODES(NUM_NODES),
        .NUM_BANKS(NRESP)
    ) resp_merge_inst (
        .clk_i(clk_i),
        .rst_i(rst_i),
        .in_valid_i(r_valid),
        .in_worker_i(r_worker),
        .in_rdata_i(r_rdata),
        .resp_valid_o(buf_resp_valid_o),
        .resp_worker_o(buf_resp_worker_o),
        .resp_rdata_o(buf_resp_rdata_o)
    );

endmodule
