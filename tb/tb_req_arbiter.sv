`timescale 1ns/1ps

module tb_req_arbiter;

    localparam int DATA_W = 64;
    localparam int ADDR_W = 64;
    localparam int NUM_NODES = 4;
    localparam int RELEASE_SET_DEPTH = 16;
    localparam int CLK_PERIOD = 10;

    localparam logic [1:0] CMD_LOAD = 2'b00;
    localparam logic [1:0] CMD_TX_ABORT = 2'b01;
    localparam logic [1:0] CMD_TX_COMMIT = 2'b10;

    logic clk_i;
    logic rst_i;

    logic [NUM_NODES-1:0] node_req_valid_i;
    logic [NUM_NODES-1:0][1:0] node_tx_signal_i;
    logic [NUM_NODES-1:0][ADDR_W-1:0] node_load_addr_i;
    logic [NUM_NODES-1:0][RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] node_req_addr_i;
    logic [NUM_NODES-1:0][RELEASE_SET_DEPTH-1:0][DATA_W-1:0] node_req_data_i;

    logic [NUM_NODES-1:0] req_valid_o;
    logic [1:0] tx_signal_o;
    logic [ADDR_W-1:0] load_addr_o;
    logic [RELEASE_SET_DEPTH-1:0] release_is_write_o;
    logic [RELEASE_SET_DEPTH-1:0] release_is_read_o;
    logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr_o;
    logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data_o;

    int failures = 0;

    req_arbiter #(
        .DATA_W(DATA_W),
        .ADDR_W(ADDR_W),
        .NUM_NODES(NUM_NODES),
        .RELEASE_SET_DEPTH(RELEASE_SET_DEPTH)
    ) dut (
        .clk_i(clk_i),
        .rst_i(rst_i),
        .node_req_valid_i(node_req_valid_i),
        .node_tx_signal_i(node_tx_signal_i),
        .node_load_addr_i(node_load_addr_i),
        .node_req_addr_i(node_req_addr_i),
        .node_req_data_i(node_req_data_i),
        .req_valid_o(req_valid_o),
        .tx_signal_o(tx_signal_o),
        .load_addr_o(load_addr_o),
        .release_is_write_o(release_is_write_o),
        .release_is_read_o(release_is_read_o),
        .release_addr_o(release_addr_o),
        .release_data_o(release_data_o)
    );

    initial clk_i = 1'b0;
    always #(CLK_PERIOD / 2) clk_i = ~clk_i;

    task automatic clear_inputs;
        node_req_valid_i = '0;
        node_tx_signal_i = '0;
        node_load_addr_i = '0;
        node_req_addr_i = '0;
        node_req_data_i = '0;
    endtask

    task automatic do_reset;
        clear_inputs();
        rst_i = 1'b1;
        repeat (2) @(posedge clk_i);
        #1 rst_i = 1'b0;
        @(negedge clk_i);
        #1;
    endtask

    task automatic check_grant(
        input int node,
        input logic [1:0] expected_cmd,
        input logic [ADDR_W-1:0] expected_load_addr,
        input string test_name
    );
        logic [NUM_NODES-1:0] expected_valid;
        expected_valid = '0;
        expected_valid[node] = 1'b1;

        if (req_valid_o !== expected_valid) begin
            $error("%s: req_valid_o expected %b, got %b",
                   test_name, expected_valid, req_valid_o);
            failures++;
        end
        if (tx_signal_o !== expected_cmd) begin
            $error("%s: tx_signal_o expected %b, got %b",
                   test_name, expected_cmd, tx_signal_o);
            failures++;
        end
        if (load_addr_o !== expected_load_addr) begin
            $error("%s: load_addr_o expected 0x%0h, got 0x%0h",
                   test_name, expected_load_addr, load_addr_o);
            failures++;
        end
    endtask

    initial begin
        $dumpfile("tb_req_arbiter.vcd");
        $dumpvars(0, tb_req_arbiter);

        rst_i = 1'b0;
        clear_inputs();

        // Reset leaves the arbiter idle with no selected node.
        do_reset();
        #1;
        if (req_valid_o !== '0) begin
            $error("idle after reset: expected no grant, got %b", req_valid_o);
            failures++;
        end

        // A single LOAD request is granted and returns its load address.
        do_reset();
        node_req_valid_i[1] = 1'b1;
        node_tx_signal_i[1] = CMD_LOAD;
        node_load_addr_i[1] = 64'h1000_0040;
        #1;
        check_grant(1, CMD_LOAD, 64'h1000_0040, "single LOAD");

        // LOAD requests must take precedence over concurrent transaction requests.
        do_reset();
        node_req_valid_i[1] = 1'b1;
        node_tx_signal_i[1] = CMD_TX_ABORT;
        node_req_valid_i[2] = 1'b1;
        node_tx_signal_i[2] = CMD_LOAD;
        node_load_addr_i[2] = 64'h2000_0080;
        #1;
        check_grant(2, CMD_LOAD, 64'h2000_0080, "LOAD priority");

        // A zero-address LOAD is still a valid grant and blocks transaction selection.
        do_reset();
        node_req_valid_i[0] = 1'b1;
        node_tx_signal_i[0] = CMD_LOAD;
        node_req_valid_i[3] = 1'b1;
        node_tx_signal_i[3] = CMD_TX_COMMIT;
        node_load_addr_i[3] = 64'h2000_00c0;
        #1;
        check_grant(0, CMD_LOAD, '0, "zero-address LOAD priority");

        // The next LOAD grant starts after the previously served node.
        do_reset();
        node_req_valid_i[0] = 1'b1;
        node_tx_signal_i[0] = CMD_LOAD;
        node_load_addr_i[0] = 64'h3000_0000;
        #1;
        check_grant(0, CMD_LOAD, 64'h3000_0000, "round-robin LOAD first");
        @(posedge clk_i);
        #1;
        @(negedge clk_i);
        node_req_valid_i[2] = 1'b1;
        node_tx_signal_i[2] = CMD_LOAD;
        node_load_addr_i[2] = 64'h3000_0080;
        #1;
        check_grant(2, CMD_LOAD, 64'h3000_0080, "round-robin LOAD next");

        // ABORT/COMMIT requests are also selected in round-robin order.
        do_reset();
        node_req_valid_i[1] = 1'b1;
        node_tx_signal_i[1] = CMD_TX_COMMIT;
        node_load_addr_i[1] = 64'h4000_0000;
        for (int entry = 0; entry < RELEASE_SET_DEPTH; entry++) begin
            node_req_addr_i[1][entry] = 64'h5000_0000 + (entry * 8);
            node_req_data_i[1][entry] = 64'h0123_4567_89ab_0000 + longint'(entry);
        end
        node_req_data_i[1][0] = 64'h0123_4567_89ab_cdef;
        node_req_valid_i[3] = 1'b1;
        node_tx_signal_i[3] = CMD_TX_ABORT;
        node_load_addr_i[3] = 64'h4000_0080;
        #1;
        check_grant(1, CMD_TX_COMMIT, 64'h4000_0000,
                    "round-robin transaction first");
        for (int entry = 0; entry < RELEASE_SET_DEPTH; entry++) begin
            if ((release_addr_o[entry] !== (64'h5000_0000 + (entry * 8))) ||
                (release_data_o[entry] !==
                    (entry == 0 ? 64'h0123_4567_89ab_cdef :
                                  64'h0123_4567_89ab_0000 + longint'(entry)))) begin
                $error("transaction payload: release entry %0d does not match node 1",
                       entry);
                failures++;
            end
        end
        if ((release_is_write_o !== 16'hcdef) ||
            (release_is_read_o !== 16'h3210)) begin
            $error("transaction release masks do not match the low release-set bits");
            failures++;
        end

        @(posedge clk_i);
        #1;
        @(negedge clk_i);
        #1;
        check_grant(3, CMD_TX_ABORT, 64'h4000_0080,
                    "round-robin transaction next");

        if (failures != 0)
            $fatal(1, "tb_req_arbiter failed with %0d error(s)", failures);

        $display("tb_req_arbiter passed");
        $finish;
    end

endmodule
