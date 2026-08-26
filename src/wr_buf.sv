module wr_buf #(
    parameter int NUM_NODES = 4,
    parameter int DATA_W = 64,
    parameter int ADDR_W = 64,
    parameter int DEPTH = 32
)(
    input logic clk_i,
    input logic rst_i,

    // Input from release_engine / router
    // Push
    input logic push_valid_i, // handshake
    input logic push_we_i,
    input logic [ADDR_W-1:0] push_addr_i,
    input logic [DATA_W-1:0] push_wdata_i,
    input logic [NUM_NODES-1:0] push_worker_i,
    input logic [1:0] push_req_type_i,

    // Output to router / controller
    output logic push_ready_o, // FIFO not full (loads that forward are always ready)

    // Input from the memory pool
    input logic mem_ready_i,
    input logic [DATA_W-1:0] mem_rdata_i,

    // Output to memory pool
    // Pop
    output logic pop_valid_o,
    output logic pop_we_o,
    output logic [ADDR_W-1:0] pop_addr_o,
    output logic [DATA_W-1:0] pop_wdata_o,
    output logic [NUM_NODES-1:0] pop_worker_o,
    output logic [1:0] pop_req_type_o,

    // Output to the nodes: memory-read completion (miss loads)
    output logic resp_valid_o,
    output logic [NUM_NODES-1:0] resp_worker_o,
    output logic [DATA_W-1:0] resp_rdata_o,

    // Output to the nodes: FORWARDED completion (hit loads)
    // Kept on a separate port so a forward and a memory-read completion
    // can co-exist in the same cycle (merged downstream).
    output logic fwd_valid_o,
    output logic [NUM_NODES-1:0] fwd_worker_o,
    output logic [DATA_W-1:0] fwd_rdata_o
);

    localparam int PTR_W = $clog2(DEPTH);

    typedef struct packed {
        logic we;
        logic [ADDR_W-1:0] addr;
        logic [DATA_W-1:0] wdata;
        logic [NUM_NODES-1:0] worker;
        logic [1:0] req_type;
    } entry_t;

    // Simple FIFO buffer
    entry_t mem [DEPTH];
    logic [PTR_W-1:0] wr_ptr;
    logic [PTR_W-1:0] rd_ptr;
    logic [PTR_W:0] count;

    logic full;
    logic empty;
    assign full = (count == DEPTH[PTR_W:0]);
    assign empty = (count == '0);

    // Registers to store in-flight memory requests
    logic req_in_flight;
    logic [NUM_NODES-1:0] inflight_worker;

    // ================ Store-to-load forwarding search ================
    // On a LOAD push, combinationally scan the resident entries newest->oldest 
    // for a WRITE to the same address. 
    
    // The youngest match is the value the load must observe.
    // All writes to an address land in this bank, so the search is
    // exhaustive. Only meaningful when the incoming beat is a load (push_we_i==0).
    logic fwd_hit;
    logic [DATA_W-1:0] fwd_match_data;
    int wb_ix; // module-scope scratch (iverilog: no locals in blocks)
    entry_t wb_e;  // whole-element scratch (field-select on indexed
                    // packed-struct arrays is rejected by iverilog)
    always_comb begin
        fwd_hit = 1'b0;
        fwd_match_data = '0;
        wb_ix = 0;
        wb_e = '0;
        for (int k = 0; k < DEPTH; k++) begin
            if (!fwd_hit && (k < int'(count))) begin
                // k-th newest resident entry = (wr_ptr - 1 - k) mod DEPTH
                wb_ix = ((int'(wr_ptr) - 1 - k) % DEPTH + DEPTH) % DEPTH;
                wb_e  = mem[wb_ix];
                if (wb_e.we && (wb_e.addr == push_addr_i)) begin
                    fwd_hit = 1'b1;
                    fwd_match_data = wb_e.wdata;
                end
            end
        end
    end

    logic is_load;
    logic load_hit;
    assign is_load  = push_valid_i && !push_we_i;
    assign load_hit = is_load && fwd_hit;   // load forwarded, not enqueued

    // A forwarded load is accepted regardless of fullness (needs no storage);
    // everything else follows FIFO fullness.
    assign push_ready_o = load_hit ? 1'b1 : !full;

    // Only writes and miss-loads are actually enqueued.
    logic do_enqueue;
    assign do_enqueue = push_valid_i && !load_hit && !full;

    // combinational output wires
    assign pop_valid_o = !empty && !req_in_flight;

    entry_t entry_to_read;
    entry_t entry_to_write;
    assign entry_to_read = mem[rd_ptr];
    assign pop_we_o = entry_to_read.we;
    assign pop_addr_o = entry_to_read.addr;
    assign pop_wdata_o = entry_to_read.wdata;
    assign pop_worker_o = entry_to_read.worker;
    assign pop_req_type_o = entry_to_read.req_type;
    always_comb begin
        entry_to_write.we = push_we_i;
        entry_to_write.addr = push_addr_i;
        entry_to_write.wdata = push_wdata_i;
        entry_to_write.worker = push_worker_i;
        entry_to_write.req_type = push_req_type_i;
    end

    // Sequential Logic
    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            wr_ptr <= '0;
            rd_ptr <= '0;
            count <= '0;
            req_in_flight <= 1'b0;
            inflight_worker <= '0;
            resp_valid_o <= 1'b0;
            resp_worker_o <= '0;
            resp_rdata_o <= '0;
            fwd_valid_o <= 1'b0;
            fwd_worker_o <= '0;
            fwd_rdata_o <= '0;
        end else begin
            // Default: clear response pulses every cycle
            resp_valid_o <= 1'b0;
            fwd_valid_o  <= 1'b0;

            // Store-to-load forward: answer a hitting load directly (1-cycle pulse)
            if (load_hit) begin
                fwd_valid_o  <= 1'b1;
                fwd_worker_o <= push_worker_i;
                fwd_rdata_o  <= fwd_match_data;
            end

            // Memory response returned, relay to node.
            // Latch data with the tag on the SAME cycle so valid/worker/rdata
            // all leave the flop together, describing the same load. Sampling
            // mem_rdata_i here (while mem_ready_i is high) captures the correct
            // value; a combinational passthrough would present next-cycle bus data.
            if (req_in_flight && mem_ready_i) begin
                req_in_flight <= 1'b0;
                resp_valid_o <= 1'b1; // one-cycle pulse
                resp_worker_o <= inflight_worker;
                resp_rdata_o <= mem_rdata_i;
            end

            // Push (writes and miss-loads only)
            if (do_enqueue) begin
                mem[wr_ptr] <= entry_to_write;
                wr_ptr <= (wr_ptr == PTR_W'(DEPTH-1)) ? '0 : wr_ptr + 1'b1;
            end

            // Pop
            // only pop when it is not empty and there are no read/writes in flight
            if (!empty && !req_in_flight) begin
                inflight_worker <= entry_to_read.worker;
                req_in_flight <= entry_to_read.we ? 1'b0 : 1'b1; // writes: no in-flight wait
                rd_ptr <= (rd_ptr == PTR_W'(DEPTH-1)) ? '0 : rd_ptr + 1'b1;
            end

            // Count
            case ({do_enqueue, !empty && !req_in_flight})
                2'b10: count <= count + 1'b1;
                2'b01: count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end

endmodule