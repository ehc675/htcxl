// bank_router
// Sits between the wr_arbiter (single serialized beat stream) and the per-bank
// wr_buf instances. Purely combinational: it steers each beat to exactly one
// bank based on low-order address interleaving, and reflects that bank's
// push_ready back to the arbiter so backpressure targets only the chosen bank.
//
//   bank = addr[BANK_W-1:0]     (low-order interleave: consecutive words
//                                spread across banks -> avoids hot-banking)
//
// The payload (we/addr/wdata/worker/req_type) is broadcast to every bank; only
// the selected bank sees push_valid asserted, so only it latches the beat.
// No state lives here -- ordering/serialization remains the arbiter's job, and
// same-address beats always resolve to the same bank, preserving per-address
// (RAW) order inside that bank's FIFO.
module bank_router #(
    parameter int DATA_W = 64,
    parameter int ADDR_W = 64,
    parameter int NUM_NODES = 4,
    parameter int NUM_BANKS = 4
)(
    // I/o with wr_arbiter (one single address read/write)
    input logic in_valid_i,
    input logic in_we_i,
    input logic [ADDR_W-1:0] in_addr_i,
    input logic [DATA_W-1:0] in_wdata_i,
    input logic [NUM_NODES-1:0] in_worker_i,
    input logic [1:0] in_req_type_i,
    output logic in_ready_o, // tells the arbiter whether it is ready

    // I/o with the NUM_BANKS wr_buf instances
    output logic [NUM_BANKS-1:0] bank_valid_o, // one-hot on the selected bank
    output logic bank_we_o, // broadcast
    output logic [ADDR_W-1:0] bank_addr_o, // broadcast
    output logic [DATA_W-1:0] bank_wdata_o, // broadcast
    output logic [NUM_NODES-1:0] bank_worker_o, // broadcast
    output logic [1:0] bank_req_type_o, // broadcast
    input logic [NUM_BANKS-1:0] bank_ready_i // per-bank push_ready_o
);

    localparam int BANK_W = (NUM_BANKS <= 1) ? 1 : $clog2(NUM_BANKS);

    // Bank select low-order address bits.
    logic [BANK_W-1:0] sel; // bank idx
    assign sel = in_addr_i[BANK_W-1:0];

    // Broadcast payload to all banks.
    assign bank_we_o = in_we_i;
    assign bank_addr_o = in_addr_i;
    assign bank_wdata_o = in_wdata_i;
    assign bank_worker_o = in_worker_i;
    assign bank_req_type_o = in_req_type_i;

    // One-hot valid demux: only the selected bank is offered the beat.
    always_comb begin
        bank_valid_o = '0;
        bank_valid_o[sel] = in_valid_i;
    end

    // Ready mux: the arbiter only advances when the targeted bank can accept.
    assign in_ready_o = bank_ready_i[sel];

endmodule