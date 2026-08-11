`timescale 1ns/1ps

// send in a sequence of LOAD/COMMIT/ABORT as a complete transaction

module tx_tb;

    localparam int DATA_W = 64;
    localparam int ADDR_W = 64;
    localparam int NUM_NODES = 4;
    localparam int RELEASE_SET_DEPTH = 16;
    localparam int CXL_TABLE_DEPTH = 32;
    localparam int BUF_DEPTH = 16;
    localparam int MEM_DEPTH = 1024;
    localparam int MEM_LATENCY = 1;
    localparam int NUM_BANKS = 4;
    localparam int BANK_W = 2;

    // Command encodings
    localparam logic [1:0] CMD_LOAD = 2'b00;
    localparam logic [1:0] CMD_TX_ABORT  = 2'b01;
    localparam logic [1:0] CMD_TX_COMMIT = 2'b10;

    logic clk, rst;

    logic [NUM_NODES-1:0] req_valid;
    logic [1:0] tx_signal;
    logic [ADDR_W-1:0] load_addr;
    logic [RELEASE_SET_DEPTH-1:0] release_is_write;
    logic [RELEASE_SET_DEPTH-1:0] release_is_read;
    logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr;
    logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data;

    logic [NUM_NODES-1:0] req_ready;

    // Controller-driven completion (TX_ABORT / TX_COMMIT)
    logic [NUM_NODES-1:0] ctrl_resp_valid;
    logic [1:0] ctrl_comp_signal;

    // wr_buf-driven completion (LOAD only)
    logic buf_resp_valid;
    logic [NUM_NODES-1:0] buf_resp_worker;
    logic [DATA_W-1:0] buf_resp_rdata;

    // Self-check bookkeeping
    integer errors = 0;

    // DUT
    top #(
        .DATA_W(DATA_W), 
        .ADDR_W(ADDR_W), 
        .NUM_NODES(NUM_NODES),
        .RELEASE_SET_DEPTH(RELEASE_SET_DEPTH), 
        .CXL_TABLE_DEPTH(CXL_TABLE_DEPTH),
        .BUF_DEPTH(BUF_DEPTH),
        .MEM_DEPTH(MEM_DEPTH), 
        .MEM_LATENCY(MEM_LATENCY)
    ) top_inst (
        .clk_i(clk),
        .rst_i(rst),
        .req_valid_i(req_valid),
        .tx_signal_i(tx_signal),
        .load_addr_i(load_addr),
        .release_is_write_i(release_is_write),
        .release_is_read_i(release_is_read),
        .release_addr_i(release_addr), 
        .release_data_i(release_data),

        .req_ready_o(req_ready),
        .ctrl_resp_valid_o(ctrl_resp_valid),
        .ctrl_comp_signal_o(ctrl_comp_signal),
        .buf_resp_valid_o(buf_resp_valid),
        .buf_resp_worker_o(buf_resp_worker),
        .buf_resp_rdata_o(buf_resp_rdata)
    );

    // Clock generation and watchdog
    initial begin
        clk = 1'b0;
        $display("[0] TB: Clock Instantiated");
        #12000;
        $display("[%0t] TB: TIMEOUT", $time);
        $finish;
    end
    always #5 clk = ~clk; // 10ns clock period


    // Passive response monitor. Encodings of ctrl_comp_signal for commit vs abort
    // are not asserted here (RTL-dependent); memory readback is the authoritative
    // protocol check. This just gives visibility into responses and load returns.
    always @(posedge clk) begin
        if (!rst) begin
            // controller response
            if (ctrl_resp_valid != '0)
                $display("[%0t] TB: MON ctrl_resp_valid=%b comp_signal=%0d", $time, ctrl_resp_valid, ctrl_comp_signal);
            // buffer response (LOAD)
            if (buf_resp_valid)
                $display("[%0t] TB: MON buf_resp worker=%b rdata=%h", $time, buf_resp_worker, buf_resp_rdata);
        end
    end

    // Find bank index of any given address
    function automatic [BANK_W-1:0] a_to_bank_idx(
        input logic [ADDR_W-1:0] a
    );
        return a[BANK_W-1:0];
    endfunction

    // Memory write check 
    // Checks mem[a] with expected value exp
    // Success commits must land the write; failed commits must leave the preload value untouched.
    task automatic expect_mem(
        input logic [ADDR_W-1:0] a,
        input logic [DATA_W-1:0] exp
    );
        logic [DATA_W-1:0] got;
        // convert raw address into bank specific address
        logic [ADDR_W-1:0] ba;
        ba = (a % MEM_DEPTH);
        case (a_to_bank_idx(a))
            2'b00: got = top_inst.g_bank[0].mem.mem[ba];
            2'b01: got = top_inst.g_bank[1].mem.mem[ba];
            2'b10: got = top_inst.g_bank[2].mem.mem[ba];
            2'b11: got = top_inst.g_bank[3].mem.mem[ba];
            default: got = 'x;
        endcase
        if (got === exp)
            $display("[%0t] TB: PASS mem[%0d] = %h", $time, a, got);
        else begin
            $display("[%0t] TB: FAIL mem[%0d] = %h (expected %h)", $time, a, got, exp);
            errors = errors + 1;
        end
    endtask

    // Drive a LOAD
    task automatic issue_load(
        input logic [NUM_NODES-1:0] node,
        input logic [ADDR_W-1:0] addr
    );
        @(negedge clk);
        while ((req_ready & node) == '0) @(negedge clk);  // wait until this node is ready

        req_valid = node;
        tx_signal = CMD_LOAD;
        load_addr = addr;
        $display("[%0t] TB: LOAD issued (node=%b addr=%0d)", $time, node, addr);

        @(posedge clk); // controller accepts here (req_valid & req_ready)
        @(negedge clk);
        req_valid = '0;
    endtask

    // Drive a ABORT
    task automatic issue_abort(
        input logic [NUM_NODES-1:0] node,
        input logic [RELEASE_SET_DEPTH-1:0] release_is_read_i, // Abort only uses read set
        input logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr_i,
        input logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data_i
    );
        @(negedge clk);
        while ((req_ready & node) == '0) @(negedge clk);  // wait until this node is ready

        req_valid = node;
        tx_signal = CMD_TX_ABORT;
        // release set
        release_is_read = release_is_read_i;
        release_addr = release_addr_i;
        release_data = release_data_i;
        $display("[%0t] TB: ABORT issued (node=%b)", $time, node);

        @(posedge clk); // controller accepts here (req_valid & req_ready)
        @(negedge clk);
        req_valid = '0;
    endtask

    // Drive a COMMIT
    task automatic issue_commit(
        input logic [NUM_NODES-1:0] node,
        input logic [RELEASE_SET_DEPTH-1:0] release_is_write_i,
        input logic [RELEASE_SET_DEPTH-1:0] release_is_read_i,
        input logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr_i,
        input logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data_i
    );
        @(negedge clk);
        while ((req_ready & node) == '0) @(negedge clk);  // wait until this node is ready

        req_valid = node;
        tx_signal = CMD_TX_COMMIT;
        // release set
        release_is_write = release_is_write_i;
        release_is_read = release_is_read_i;
        release_addr = release_addr_i;
        release_data = release_data_i;
        $display("[%0t] TB: COMMIT issued (node=%b)", $time, node);

        @(posedge clk); // controller accepts here (req_valid & req_ready)
        @(negedge clk);
        req_valid = '0;
    endtask
 

    initial begin
        logic [RELEASE_SET_DEPTH-1:0] release_is_write_t;
        logic [RELEASE_SET_DEPTH-1:0] release_is_read_t;
        logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr_t;
        logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data_t;

        // Init all inputs
        req_valid = '0; 
        tx_signal = 0; 
        load_addr = 0;
        release_is_write = 0; 
        release_is_read = 0;
        release_addr = 0; 
        release_data = 0;

        // Reset
        rst = 1'b1;
        repeat (4) @(posedge clk);
        rst = 1'b0;
        @(posedge clk);

        // Load array into memory
        top_inst.g_bank[0].mem.mem[0] = 64'hDEAD_BEEF_0000_0000;
        top_inst.g_bank[1].mem.mem[0] = 64'hDEAD_BEEF_0000_0001;
        top_inst.g_bank[2].mem.mem[0] = 64'hDEAD_BEEF_0000_0002;
        top_inst.g_bank[3].mem.mem[0] = 64'hDEAD_BEEF_0000_0003;
        top_inst.g_bank[0].mem.mem[1] = 64'hDEAD_BEEF_0000_0004;
        top_inst.g_bank[1].mem.mem[1] = 64'hDEAD_BEEF_0000_0005;
        top_inst.g_bank[2].mem.mem[1] = 64'hDEAD_BEEF_0000_0006;
        top_inst.g_bank[3].mem.mem[1] = 64'hDEAD_BEEF_0000_0007;
        $display("[%0t] TB: memory preloaded", $time);
        
        $display("[%0t] debug: preloaded memory bank[0], mem[0] = %h", $time, top_inst.g_bank[0].mem.mem[0]);
        $display("[%0t] debug: preloaded memory bank[0], mem[1] = %h", $time, top_inst.g_bank[0].mem.mem[1]);
        $display("[%0t] debug: preloaded memory bank[1], mem[0] = %h", $time, top_inst.g_bank[1].mem.mem[0]);
        $display("[%0t] debug: preloaded memory bank[1], mem[1] = %h", $time, top_inst.g_bank[1].mem.mem[1]);
        $display("[%0t] debug: preloaded memory bank[2], mem[0] = %h", $time, top_inst.g_bank[2].mem.mem[0]);
        $display("[%0t] debug: preloaded memory bank[2], mem[1] = %h", $time, top_inst.g_bank[2].mem.mem[1]);
        $display("[%0t] debug: preloaded memory bank[3], mem[0] = %h", $time, top_inst.g_bank[3].mem.mem[0]);
        $display("[%0t] debug: preloaded memory bank[3], mem[1] = %h", $time, top_inst.g_bank[3].mem.mem[1]);


        // Issue load on the entire array
        issue_load(4'b0001, 64'd0);
        issue_load(4'b0001, 64'd1);
        issue_load(4'b0001, 64'd2);
        issue_load(4'b0001, 64'd3);
        issue_load(4'b0001, 64'd4);
        issue_load(4'b0001, 64'd5);
        issue_load(4'b0001, 64'd6);
        issue_load(4'b0001, 64'd7);

        // construct release set
        for(int i = 0; i < 8; i++) begin
            release_is_write_t[i] = 1'b1;
            release_is_read_t[i] = 1'b0;
            release_addr_t[i] = i;
            release_data_t[i] = 64'hDEAD_BEEF_0000_0000 + i + 16; // array addition
        end

        // commit (write)
        issue_commit(4'b0001, release_is_write_t, release_is_read_t, release_addr_t, release_data_t);
        
        repeat (50) @(posedge clk);

        // Check memory content
        expect_mem(64'h0000_0000_0000_0000, 64'hDEAD_BEEF_0000_0010);
        expect_mem(64'h0000_0000_0000_0001, 64'hDEAD_BEEF_0000_0011);
        expect_mem(64'h0000_0000_0000_0002, 64'hDEAD_BEEF_0000_0012);
        expect_mem(64'h0000_0000_0000_0003, 64'hDEAD_BEEF_0000_0013);
        expect_mem(64'h0000_0000_0000_0004, 64'hDEAD_BEEF_0000_0014);
        expect_mem(64'h0000_0000_0000_0005, 64'hDEAD_BEEF_0000_0015);
        expect_mem(64'h0000_0000_0000_0006, 64'hDEAD_BEEF_0000_0016);
        expect_mem(64'h0000_0000_0000_0007, 64'hDEAD_BEEF_0000_0017);

    end

    // Waveforms
    initial begin
        $dumpfile("tx_tb.vcd");
        $dumpvars(0, tx_tb);
    end

endmodule