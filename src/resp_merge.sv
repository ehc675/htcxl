// resp_merge
// Collapses the NUM_BANKS independent read-response streams (one per wr_buf)
// down to the single node-facing response port.

// Note that the assumed memory latency in the SRAM allows us to use 1-deep registers
// to hold responses

module resp_merge #(
    parameter int DATA_W = 64,
    parameter int NUM_NODES = 4,
    parameter int NUM_BANKS = 4
)(
    input logic clk_i,
    input logic rst_i,

    // ---- From the NUM_BANKS wr_buf instances
    // intercept each buffer's output to the nodes
    input logic [NUM_BANKS-1:0] in_valid_i,
    input logic [NUM_BANKS-1:0][NUM_NODES-1:0] in_worker_i,
    input logic [NUM_BANKS-1:0][DATA_W-1:0] in_rdata_i,

    // ---- To the nodes (single port) ----
    output logic resp_valid_o,
    output logic [NUM_NODES-1:0] resp_worker_o,
    output logic [DATA_W-1:0] resp_rdata_o
);

    localparam int BSEL_W = (NUM_BANKS <= 1) ? 1 : $clog2(NUM_BANKS);

    // Per-bank 1-deep holding registers (data gated by hold_valid_q).
    logic [NUM_BANKS-1:0] hold_valid_q;
    logic [NUM_BANKS-1:0][NUM_NODES-1:0] hold_worker_q;
    logic [NUM_BANKS-1:0][DATA_W-1:0] hold_rdata_q;

    // Round-robin priority pointer.
    logic [BSEL_W-1:0] rr_ptr_q;

    // A bank is requesting service if it pulses now or has a held response.
    logic [NUM_BANKS-1:0] req;
    assign req = in_valid_i | hold_valid_q;

    // Round-robin winner: first requesting bank at/after rr_ptr, wrapping.
    logic [BSEL_W-1:0] win_idx;
    logic win_found;
    always_comb begin
        win_idx = rr_ptr_q;
        win_found = 1'b0;
        // iterate through banks to find winner
        for (int k = 0; k < NUM_BANKS; k++) begin
            if (!win_found && req[(int'(rr_ptr_q) + k) % NUM_BANKS]) begin
                win_idx = BSEL_W'((int'(rr_ptr_q) + k) % NUM_BANKS);
                win_found = 1'b1;
            end
        end
    end

    always_ff @(posedge clk_i or posedge rst_i) begin
        if (rst_i) begin
            hold_valid_q <= '0;
            rr_ptr_q <= '0;
            resp_valid_o <= 1'b0;
            resp_worker_o <= '0;
            resp_rdata_o <= '0;
        end else begin
            resp_valid_o <= 1'b0; // default: one-cycle pulse

            // Invariant guard (sim-only): held + fresh in the same cycle means a
            // dropped response. Should be impossible under 1-beat/cycle issue.
            for (int b = 0; b < NUM_BANKS; b++) begin
                if (in_valid_i[b] && hold_valid_q[b])
                    $error("resp_merge: bank %0d response overflow (held+fresh)", b);
            end

            // Capture incoming pulses that are NOT being forwarded this cycle.
            for (int b = 0; b < NUM_BANKS; b++) begin
                if (in_valid_i[b] && !(win_found && (win_idx == BSEL_W'(b)))) begin
                    hold_valid_q[b] <= 1'b1;
                    hold_worker_q[b] <= in_worker_i[b];
                    hold_rdata_q[b] <= in_rdata_i[b];
                end
            end

            // Drive output signals for the winner (held value takes priority over a fresh pulse,
            // though by the invariant above they never coincide).
            if (win_found) begin
                resp_valid_o <= 1'b1;
                resp_worker_o <= hold_valid_q[win_idx] ? hold_worker_q[win_idx] : in_worker_i[win_idx];
                resp_rdata_o <= hold_valid_q[win_idx] ? hold_rdata_q[win_idx] : in_rdata_i[win_idx];
                if (hold_valid_q[win_idx])
                    hold_valid_q[win_idx] <= 1'b0;

                rr_ptr_q <= BSEL_W'((int'(win_idx) + 1) % NUM_BANKS);
            end
        end
    end

endmodule