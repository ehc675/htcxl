`timescale 1ns / 1ps

module req_arbiter #(
    parameter int DATA_W = 64,
    parameter int ADDR_W = 64,

    parameter int NUM_NODES = 4,
    parameter int RELEASE_SET_DEPTH = 16, // max 16 variables per transaction
    parameter int CXL_TABLE_DEPTH = 32, // max 32 entires in cxl table

)(
    input logic clk_i,
    input logic rst_i,

    // Node Requests
    input logic [NUM_NODES-1:0]  node_req_valid_i, // handshake signal specific for each node, also encodes the requesting node's id
    input logic [NUM_NODES-1:0][1:0]  node_tx_signal_i, // 00: CMD_LOAD, 01: CMD_TX_ABORT, 10: CMD_TX_COMMIT
    input logic [NUM_NODES-1:0][ADDR_W-1:0]  node_load_addr_i, // Used for CMD_LOAD

    input logic [NUM_NODES-1:0][ADDR_W-1:0]  node_req_addr_i,
    input logic [NUM_NODES-1:0][DATA_W-1:0]  node_req_data_i,

    output logic [NUM_NODES-1:0] req_valid_o,
    output logic [1:0] tx_signal_o,
    output logic [ADDR_W-1:0] load_addr_o,

    output logic [RELEASE_SET_DEPTH-1:0] release_is_write_o,
    output logic [RELEASE_SET_DEPTH-1:0] release_is_read_o,
    output logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr_o,
    output logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data_o,
);

    logic req_valid = 1'b0;
    logic tx_signal = 2'b00;
    logic load_addr = '0;

    logic release_is_write = '0;
    logic release_is_read = '0;
    logic [ADDR_W-1:0] release_addr = '0;
    logic [DATA_W-1:0] release_data = '0;

    int cnt = 0;

    typedef enum logic [1:0] {
        CMD_LOAD = 2'b00,
        CMD_TX_ABORT  = 2'b01,
        CMD_TX_COMMIT = 2'b10
    } cxl_cmd_t;

    always_comb begin
        req_valid_o = '0;
        tx_signal_o = '0;
        load_addr_o = '0;

        release_is_write_o = '0;
        release_is_read_o = '0;
        release_addr_o = '0;
        release_data_o = '0;

        // 1) priority group: LOADs
        for (int k = 0; k < NUM_NODES; k++) begin
            int idx;
            idx = (rr_ptr_q + k) % NUM_NODES;

            if (node_req_valid_i[idx] && (node_tx_signal_i[idx] == CMD_LOAD)) begin
                req_valid_o[idx] = 1'b1;
                tx_signal_o = node_tx_signal_i[idx];
                load_addr_o = node_load_addr_i[idx];
                rr_ptr_d = (idx + 1) % NUM_NODES;
                break;
            end
        end

        // 2) if no load is found, look for ABORT/COMMIT in round-robin order
        if (tx_signal_o == '0) begin
            for (int k = 0; k < NUM_NODES; k++) begin
                int idx;
                idx = (rr_ptr_q + k) % NUM_NODES;

                if (node_req_valid_i[idx] &&
                    (node_tx_signal_i[idx] == CMD_TX_ABORT ||
                    node_tx_signal_i[idx] == CMD_TX_COMMIT)) begin
                    req_valid_o[idx] = 1'b1;
                    tx_signal_o = node_tx_signal_i[idx];
                    load_addr_o = node_load_addr_i[idx];

                    release_is_write_o = node_req_data_i[idx];
                    release_is_read_o = ~node_req_data_i[idx];
                    release_addr_o = node_req_addr_i[idx];
                    release_data_o = node_req_data_i[idx];

                    rr_ptr_d = (idx + 1) % NUM_NODES;
                    break;
                end
            end
        end
    end

    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i)
            rr_ptr_q <= '0;
        else
            rr_ptr_q <= rr_ptr_d;
    end



//Add Coalescing logic at the memory level instead, use normal non-fifo input for arbiter as nodes hold request till they are serviced

//Release set only for abort commit 
//abort - remove all release set
//commit - remove all release set and write to memory pool 
endmodule