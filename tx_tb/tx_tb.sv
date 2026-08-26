`timescale 1ns/1ps

// ---------------------------------------------------------------------------
// General single-node transaction testbench with latency characterization.
//
// Primitives (run_load / run_abort / run_commit) drive one transaction, block
// until it completes, verify it, and record its latency. Each record is also
// streamed live to a CSV file (tx_latency.csv) for downstream analysis in C /
// YACSIM / etc.

// Latency = clock cycles from the accept edge (req_valid & req_ready) to the
// edge at which the TB observes the response. Responses are registered, so all
// three transaction types carry the same +1 observation offset; the numbers
// are directly comparable to each other.

// The controller runs a single machine-wide in-flight transaction, so a node
// can never have two outstanding at once => blocking-per-transaction is the
// correct structure for latency measurement.

// Each test resets and preloads its own state, so tests are independent and may
// be reordered or commented out freely. To characterize a parameter, wrap a
// test in a loop and vary the swept value; every transaction is tagged with a
// `setsize` field and stamped with MEM_LATENCY in the CSV so runs combine.
// ---------------------------------------------------------------------------

module tx_tb;

    // specs
    localparam int DATA_W = 64;
    localparam int ADDR_W = 64;
    localparam int NUM_NODES = 4;
    localparam int RELEASE_SET_DEPTH = 16;
    localparam int CXL_TABLE_DEPTH = 32;
    localparam int BUF_DEPTH = 16;
    localparam int MEM_DEPTH = 1024;
    localparam int MEM_LATENCY = 1; // sweep this across builds for a latency curve
    localparam int NUM_BANKS = 4;
    localparam int BANK_W = 2;

    // Command encodings
    localparam logic [1:0] CMD_LOAD = 2'b00;
    localparam logic [1:0] CMD_TX_ABORT  = 2'b01;
    localparam logic [1:0] CMD_TX_COMMIT = 2'b10;

    // Transaction type codes
    localparam int TX_LOAD = 0;
    localparam int TX_ABORT = 1;
    localparam int TX_COMMIT = 2;

    // Tunables
    localparam int TIMEOUT_CYCLES = 400;
    localparam int COMMIT_DRAIN = 50;
    localparam int ABORT_DRAIN = 10;
    localparam int MAX_REC = 4096; // maximum number of single transactions tested in one run of the tb

    logic clk, rst;

    // DUT interface
    logic [NUM_NODES-1:0] req_valid;
    logic [1:0] tx_signal;
    logic [ADDR_W-1:0] load_addr;
    logic [RELEASE_SET_DEPTH-1:0] release_is_write;
    logic [RELEASE_SET_DEPTH-1:0] release_is_read;
    logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr;
    logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data;
    
    // req_ready is the output to hosts/nodes, tell the host that CXL controller is ready to accept a new request from specifc nodes
    logic [NUM_NODES-1:0] req_ready;

    // Controller-driven completion (TX_ABORT / TX_COMMIT)
    logic [NUM_NODES-1:0] ctrl_resp_valid;
    logic [1:0] ctrl_comp_signal;

    // wr_buf-driven completion (LOAD only)
    logic buf_resp_valid;
    logic [NUM_NODES-1:0] buf_resp_worker;
    logic [DATA_W-1:0] buf_resp_rdata;

    // error count
    integer errors = 0;

    // cycle count
    int cycle_count = 0;
    always @(posedge clk) begin
        if (rst) cycle_count <= 0;
        else cycle_count <= cycle_count + 1;
    end

    // Parallel arrays to document the results
    int rec_type [MAX_REC];
    int rec_size [MAX_REC];
    int rec_cycles [MAX_REC];
    bit rec_pass [MAX_REC];
    int rec_n = 0; // pointer
    integer csv_fd = 0; // file descriptor

    // convert integer t to the transaction type name string
    function automatic string type_name(input int t);
        case (t)
            TX_LOAD: return "LOAD";
            TX_ABORT: return "ABORT";
            TX_COMMIT: return "COMMIT";
            default: return "UNK";
        endcase
    endfunction

    // record results in parallel arrays
    task automatic record_tx(input int ttype, input string label, input int setsize, input int cyc, input bit pass);
        if (rec_n < MAX_REC) begin
            rec_type[rec_n] = ttype;
            rec_size[rec_n] = setsize;
            rec_cycles[rec_n] = cyc;
            rec_pass[rec_n] = pass;
        end
        if (csv_fd != 0)
            $fdisplay(csv_fd, "%0d,%0d,%s,%s,%0d,%0d,%0d", rec_n, MEM_LATENCY, type_name(ttype), label, setsize, cyc, pass);
        rec_n = rec_n + 1;
    endtask

    // count the number of bits in release set to determine release set depth
    function automatic int count_bits(input logic [RELEASE_SET_DEPTH-1:0] v);
        int n;
        n = 0;
        for (int i = 0; i < RELEASE_SET_DEPTH; i++)
            if (v[i]) n++;
        return n;
    endfunction

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

    // Clock + watchdog
    initial begin
        clk = 1'b0;
        $display("[0] TB: Clock Instantiated");
        #200000;
        $display("[%0t] TB: TIMEOUT (watchdog)", $time);
        $finish;
    end
    always #5 clk = ~clk;

    // Passive monitor (visibility only)
    always @(posedge clk) begin
        if (!rst) begin
            if (ctrl_resp_valid != '0)
                $display("[%0t] TB: MON ctrl_resp_valid=%b comp_signal=%0d", $time, ctrl_resp_valid, ctrl_comp_signal);
            if (buf_resp_valid)
                $display("[%0t] TB: MON buf_resp worker=%b rdata=%h", $time, buf_resp_worker, buf_resp_rdata);
        end
    end

    // ------------------------------------------------------------------
    // Memory helpers
    // ------------------------------------------------------------------
    // convert an address to its corresponding bank (lowest BANK_W bits)
    function automatic [BANK_W-1:0] a_to_bank_idx(input logic [ADDR_W-1:0] a);
        return a[BANK_W-1:0]; // truncate
    endfunction

    // preload memory into memory pool
    task automatic mem_preload(input logic [ADDR_W-1:0] a, input logic [DATA_W-1:0] d);
        logic [ADDR_W-1:0] row;
        row = (a % MEM_DEPTH);
        case (a_to_bank_idx(a))
            2'b00: top_inst.g_bank[0].mem.mem[row] = d;
            2'b01: top_inst.g_bank[1].mem.mem[row] = d;
            2'b10: top_inst.g_bank[2].mem.mem[row] = d;
            2'b11: top_inst.g_bank[3].mem.mem[row] = d;
        endcase
    endtask

    // verify memory contwnt
    task automatic expect_mem(input logic [ADDR_W-1:0] a, input logic [DATA_W-1:0] exp);
        logic [DATA_W-1:0] got;
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

    // ------------------------------------------------------------------
    // DUT control
    // ------------------------------------------------------------------
    task automatic reset_dut;
        @(negedge clk);
        rst = 1'b1;
        req_valid = '0; tx_signal = '0; load_addr = '0;
        release_is_write = '0; release_is_read = '0;
        release_addr = '0; release_data = '0;
        repeat (4) @(posedge clk);
        @(negedge clk);
        rst = 1'b0;
        @(posedge clk);
    endtask

    // blocks until the controller signals a response for the caller's node, or until a watchdog fires, 
    // and hands back three things: when it happened, what the completion code was, and whether it timed out.
    // Note that it checks ctrl_resp_valid, which is sent from the controller, hence it is used for commit and abort
    task automatic wait_ctrl_resp(input logic [NUM_NODES-1:0] node, output int done_cycle, output logic [1:0] comp, output bit timed_out);
        int guard;
        guard = 0;
        timed_out = 1'b0;

        forever begin
            @(posedge clk);
            if ((ctrl_resp_valid & node) != '0) break;
            guard++;
            if (guard > TIMEOUT_CYCLES) begin
                timed_out = 1'b1;
                break;
            end
        end

        done_cycle = cycle_count;
        comp = ctrl_comp_signal;
    endtask

    // ------------------------------------------------------------------
    // Transaction primitives (blocking, latency-measuring)
    // ------------------------------------------------------------------
    task automatic run_load(input logic [NUM_NODES-1:0] node,
                            input logic [ADDR_W-1:0] addr,
                            input logic [DATA_W-1:0] exp,
                            input string label);
        int t0, t1, guard;
        bit ok;
        @(negedge clk);

        // wait until available then drive input wires
        while ((req_ready & node) == '0) @(negedge clk);
        req_valid = node;
        tx_signal = CMD_LOAD;
        load_addr = addr;
        @(posedge clk);
        t0 = cycle_count;
        @(negedge clk);
        req_valid = '0;

        guard = 0; // timeout guard
        forever begin
            @(posedge clk);
            // not using wait_ctrl_resp function because LOAD does not take response from controller but rather the buffer
            if (buf_resp_valid && (buf_resp_worker == node)) break;
            guard++;
            if (guard > TIMEOUT_CYCLES) begin
                $display("[%0t] TB: FAIL [%s] LOAD timeout (node=%b addr=%0d)", $time, label, node, addr);
                errors = errors + 1;
                record_tx(TX_LOAD, label, 1, -1, 1'b0);
                return;
            end
        end
        t1 = cycle_count;

        // check if loaded data matches expected value
        ok = (buf_resp_rdata === exp);
        if (ok)
            $display("[%0t] TB: PASS [%s] LOAD node=%b addr=%0d data=%h  lat=%0d cyc", $time, label, node, addr, buf_resp_rdata, t1 - t0);
        else begin
            $display("[%0t] TB: FAIL [%s] LOAD node=%b addr=%0d data=%h (exp %h)  lat=%0d cyc", $time, label, node, addr, buf_resp_rdata, exp, t1 - t0);
            errors = errors + 1;
        end
        record_tx(TX_LOAD, label, 1, t1 - t0, ok);
    endtask

    task automatic run_commit(input logic [NUM_NODES-1:0] node,
                              input logic [RELEASE_SET_DEPTH-1:0] is_write,
                              input logic [RELEASE_SET_DEPTH-1:0] is_read,
                              input logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] raddr,
                              input logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] rdata,
                              input string label);
        int t0, t1, wsz;
        logic [1:0] comp;
        bit to;
        wsz = count_bits(is_write);

        @(negedge clk);
        while ((req_ready & node) == '0) @(negedge clk);
        req_valid = node;
        tx_signal = CMD_TX_COMMIT;
        release_is_write = is_write;
        release_is_read = is_read;
        release_addr = raddr;
        release_data = rdata;
        @(posedge clk);
        t0 = cycle_count;
        @(negedge clk);
        req_valid = '0;

        wait_ctrl_resp(node, t1, comp, to);
        if (to) begin
            $display("[%0t] TB: FAIL [%s] COMMIT timeout (node=%b)", $time, label, node);
            errors = errors + 1;
            record_tx(TX_COMMIT, label, wsz, -1, 1'b0);
        end else begin
            $display("[%0t] TB: DONE [%s] COMMIT node=%b wset=%0d comp=%0d  lat=%0d cyc",
                    $time, label, node, wsz, comp, t1 - t0);
            record_tx(TX_COMMIT, label, wsz, t1 - t0, 1'b1);
        end
        release_is_write = '0; release_is_read = '0;
    endtask

    task automatic run_abort(input logic [NUM_NODES-1:0] node,
                             input logic [RELEASE_SET_DEPTH-1:0] is_read,
                             input logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] raddr,
                             input logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] rdata,
                             input string label);
        int t0, t1, rsz;
        logic [1:0] comp;
        bit to;
        rsz = count_bits(is_read);

        @(negedge clk);
        while ((req_ready & node) == '0) @(negedge clk);
        req_valid = node;
        tx_signal = CMD_TX_ABORT;
        release_is_write = '0;
        release_is_read  = is_read;
        release_addr     = raddr;
        release_data     = rdata;
        @(posedge clk);
        t0 = cycle_count;
        @(negedge clk);
        req_valid = '0;

        wait_ctrl_resp(node, t1, comp, to);
        if (to) begin
            $display("[%0t] TB: FAIL [%s] ABORT timeout (node=%b)", $time, label, node);
            errors = errors + 1;
            record_tx(TX_ABORT, label, rsz, -1, 1'b0);
        end else begin
            $display("[%0t] TB: DONE [%s] ABORT node=%b rset=%0d comp=%0d  lat=%0d cyc",
                    $time, label, node, rsz, comp, t1 - t0);
            record_tx(TX_ABORT, label, rsz, t1 - t0, 1'b1);
        end
        release_is_read = '0;
    endtask

    // Convenience wrappers
    task automatic run_commit_single(input logic [NUM_NODES-1:0] node,
                                     input logic [ADDR_W-1:0] addr,
                                     input logic [DATA_W-1:0] data,
                                     input string label);
        logic [RELEASE_SET_DEPTH-1:0] iw, ir;
        logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] ra;
        logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] rd;
        iw = '0; ir = '0; ra = '0; rd = '0;
        iw[0] = 1'b1; ra[0] = addr; rd[0] = data;
        run_commit(node, iw, ir, ra, rd, label);
    endtask

    task automatic run_abort_single(input logic [NUM_NODES-1:0] node,
                                    input logic [ADDR_W-1:0]     addr,
                                    input string                 label);
        logic [RELEASE_SET_DEPTH-1:0] ir;
        logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] ra;
        logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] rd;
        ir = '0; ra = '0; rd = '0;
        ir[0] = 1'b1; ra[0] = addr;
        run_abort(node, ir, ra, rd, label);
    endtask





















    // ==================================================================
    // TESTS  (each self-contained; comment out freely in main)
    // ==================================================================
    task automatic test_single_load;
        $display("\n=== TEST: single load ===");
        reset_dut();
        mem_preload(0, 64'hDEAD_BEEF_0000_0000);
        run_load(4'b0001, 0, 64'hDEAD_BEEF_0000_0000, "single_load");
    endtask

    task automatic test_single_abort;
        $display("\n=== TEST: single abort ===");
        reset_dut();
        mem_preload(2, 64'hDEAD_BEEF_0000_0002);
        run_abort_single(4'b0001, 2, "single_abort");
        repeat (ABORT_DRAIN) @(posedge clk);
        expect_mem(2, 64'hDEAD_BEEF_0000_0002);   // abort must not change memory
    endtask

    task automatic test_single_commit;
        logic [DATA_W-1:0] wdata;
        $display("\n=== TEST: single commit ===");
        reset_dut();
        wdata = 64'h0000_C0FFEE_0000_0005;
        mem_preload(5, 64'hDEAD_BEEF_0000_0005);
        run_commit_single(4'b0001, 5, wdata, "single_commit");
        repeat (COMMIT_DRAIN) @(posedge clk);
        expect_mem(5, wdata);                      // commit must land the write
    endtask

    task automatic test_array_access;
        logic [RELEASE_SET_DEPTH-1:0] iw, ir;
        logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] ra;
        logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] rd;
        $display("\n=== TEST: array access (load + commit) ===");
        reset_dut();
        for (int a = 0; a < 8; a++)
            mem_preload(a, 64'hDEAD_BEEF_0000_0000 + a);
        for (int a = 0; a < 8; a++)
            run_load(4'b0001, a, 64'hDEAD_BEEF_0000_0000 + a, "array_load");
        iw = '0; ir = '0; ra = '0; rd = '0;
        for (int i = 0; i < 8; i++) begin
            iw[i] = 1'b1; ra[i] = i; rd[i] = 64'hDEAD_BEEF_0000_0000 + i + 16;
        end
        run_commit(4'b0001, iw, ir, ra, rd, "array_commit");
        repeat (COMMIT_DRAIN) @(posedge clk);
        for (int a = 0; a < 8; a++)
            expect_mem(a, 64'hDEAD_BEEF_0000_0000 + a + 16);
    endtask

    // ---- extra example characterization tests ----

    // Baseline fixed overhead: commit with an empty write set (intercept term
    // for latency = a + b*N).
    task automatic test_commit_empty;
        logic [RELEASE_SET_DEPTH-1:0] iw, ir;
        logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] ra;
        logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] rd;
        $display("\n=== TEST: empty commit (baseline overhead) ===");
        reset_dut();
        iw = '0; ir = '0; ra = '0; rd = '0;
        run_commit(4'b0001, iw, ir, ra, rd, "commit_empty");
    endtask

    // Write-set-size sweep: N = 1,2,4,8,16. Gives latency vs write-set size,
    // i.e. the per-entry commit cost (slope) plus fixed overhead (intercept).
    task automatic test_commit_size_sweep;
        logic [RELEASE_SET_DEPTH-1:0] iw, ir;
        logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] ra;
        logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] rd;
        int N;
        $display("\n=== TEST: commit write-set size sweep ===");
        for (int k = 0; k < 5; k++) begin
            N = (1 << k);                         // 1,2,4,8,16
            reset_dut();
            iw = '0; ir = '0; ra = '0; rd = '0;
            for (int i = 0; i < N; i++) begin
                iw[i] = 1'b1;
                ra[i] = i;
                rd[i] = 64'hA5A5_0000_0000_0000 + (k << 8) + i;
            end
            run_commit(4'b0001, iw, ir, ra, rd, "commit_size");
            repeat (COMMIT_DRAIN) @(posedge clk);
            for (int i = 0; i < N; i++)
                expect_mem(i, 64'hA5A5_0000_0000_0000 + (k << 8) + i);
        end
    endtask

    // Address folding / coverage: k and k+MEM_DEPTH alias to the same (bank,row).
    // Closes the coverage gap where every test used unique (bank,row) pairs.
    task automatic test_alias_collision;
        $display("\n=== TEST: alias collision (k vs k+MEM_DEPTH) ===");
        reset_dut();
        mem_preload(0, 64'hFACE_0000_0000_0000);
        run_load(4'b0001, MEM_DEPTH, 64'hFACE_0000_0000_0000, "alias_load"); // reads mem[0] bank0
    endtask

    // Bank hotspot: 4 writes all landing in bank 0 (rows 0,4,8,12). Compare its
    // COMMIT latency against test_commit_spread4 to see per-bank serialization.
    // Difference is expected to be ~0 at MEM_LATENCY=1 and to grow with latency.
    task automatic test_commit_hotspot4;
        logic [RELEASE_SET_DEPTH-1:0] iw, ir;
        logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] ra;
        logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] rd;
        $display("\n=== TEST: commit hotspot (4 writes, one bank) ===");
        reset_dut();
        iw = '0; ir = '0; ra = '0; rd = '0;
        for (int i = 0; i < 4; i++) begin
            iw[i] = 1'b1; ra[i] = i*4; rd[i] = 64'hB00B_0000_0000_0000 + i;  // 0,4,8,12 -> bank0
        end
        run_commit(4'b0001, iw, ir, ra, rd, "commit_hotspot4");
        repeat (COMMIT_DRAIN) @(posedge clk);
        for (int i = 0; i < 4; i++)
            expect_mem(i*4, 64'hB00B_0000_0000_0000 + i);
    endtask

    // Bank spread: 4 writes across banks 0..3 (addrs 0,1,2,3). Baseline for the
    // hotspot comparison above.
    task automatic test_commit_spread4;
        logic [RELEASE_SET_DEPTH-1:0] iw, ir;
        logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] ra;
        logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] rd;
        $display("\n=== TEST: commit spread (4 writes, four banks) ===");
        reset_dut();
        iw = '0; ir = '0; ra = '0; rd = '0;
        for (int i = 0; i < 4; i++) begin
            iw[i] = 1'b1; ra[i] = i; rd[i] = 64'hCAFE_0000_0000_0000 + i;    // 0,1,2,3 -> banks 0..3
        end
        run_commit(4'b0001, iw, ir, ra, rd, "commit_spread4");
        repeat (COMMIT_DRAIN) @(posedge clk);
        for (int i = 0; i < 4; i++)
            expect_mem(i, 64'hCAFE_0000_0000_0000 + i);
    endtask

    // Store-to-load forwarding: commit a write, then load the same address with
    // minimal drain so the value may still be in wr_buf. Latency distinguishes
    // the forwarded fast path from a settled memory read. (Timing-dependent.)
    task automatic test_stl_forwarding;
        logic [DATA_W-1:0] wd;
        $display("\n=== TEST: store-to-load forwarding ===");
        reset_dut();
        mem_preload(3, 64'hDEAD_BEEF_0000_0003);
        wd = 64'h5555_0000_0000_0003;
        run_commit_single(4'b0001, 3, wd, "stl_commit");
        run_load(4'b0001, 3, wd, "stl_load");   // no long drain: catch the value early
    endtask

    // ------------------------------------------------------------------
    // Report
    // ------------------------------------------------------------------
    task automatic report;
        int i, t, c;
        int cnt_l, cnt_a, cnt_c;
        int sum_l, sum_a, sum_c;
        int min_l, min_a, min_c;
        int max_l, max_a, max_c;
        int pass_cnt;
        cnt_l=0; cnt_a=0; cnt_c=0; sum_l=0; sum_a=0; sum_c=0;
        min_l=1<<30; min_a=1<<30; min_c=1<<30; max_l=0; max_a=0; max_c=0; pass_cnt=0;
        for (i = 0; i < rec_n; i++) begin
            t = rec_type[i]; c = rec_cycles[i];
            if (rec_pass[i]) pass_cnt++;
            if (c >= 0) begin
                case (t)
                    TX_LOAD:   begin cnt_l++; sum_l+=c; if(c<min_l)min_l=c; if(c>max_l)max_l=c; end
                    TX_ABORT:  begin cnt_a++; sum_a+=c; if(c<min_a)min_a=c; if(c>max_a)max_a=c; end
                    TX_COMMIT: begin cnt_c++; sum_c+=c; if(c<min_c)min_c=c; if(c>max_c)max_c=c; end
                endcase
            end
        end
        $display("\n==================== LATENCY REPORT ====================");
        $display(" MEM_LATENCY = %0d", MEM_LATENCY);
        $display(" per-transaction (cycles):");
        for (i = 0; i < rec_n; i++)
            $display("   #%0d  %s  size=%0d  lat=%0d  %s",
                    i, type_name(rec_type[i]), rec_size[i], rec_cycles[i],
                    rec_pass[i] ? "OK" : "FAIL");
        $display(" per-type (cycles):");
        if (cnt_l) $display("   LOAD  : n=%0d  min=%0d  avg=%0d  max=%0d", cnt_l, min_l, sum_l/cnt_l, max_l);
        if (cnt_a) $display("   ABORT : n=%0d  min=%0d  avg=%0d  max=%0d", cnt_a, min_a, sum_a/cnt_a, max_a);
        if (cnt_c) $display("   COMMIT: n=%0d  min=%0d  avg=%0d  max=%0d", cnt_c, min_c, sum_c/cnt_c, max_c);
        $display(" transactions: %0d   completed-OK: %0d   check errors: %0d", rec_n, pass_cnt, errors);
        if (errors == 0) $display(" RESULT: ALL CHECKS PASSED");
        else             $display(" RESULT: %0d CHECK(S) FAILED", errors);
        $display(" CSV written to tx_latency.csv");
        $display("=======================================================\n");
    endtask

























    // ------------------------------------------------------------------
    // Main: pick tests to run here.
    // ------------------------------------------------------------------
    initial begin
        req_valid = '0; tx_signal = '0; load_addr = '0;
        release_is_write = '0; release_is_read = '0;
        release_addr = '0; release_data = '0;
        rst = 1'b1;

        csv_fd = $fopen("tx_latency.csv", "w");
        if (csv_fd != 0)
            $fdisplay(csv_fd, "idx,mem_latency,type,label,setsize,latency,pass");

        repeat (2) @(posedge clk);

        // ---- test selection ----
        test_single_load();
        test_single_abort();
        test_single_commit();
        // test_array_access();
        // test_commit_empty();
        // test_commit_size_sweep();
        // test_alias_collision();
        // test_commit_spread4();
        // test_commit_hotspot4();
        // test_stl_forwarding();

        report();
        if (csv_fd != 0) $fclose(csv_fd);
        $finish;
    end

    initial begin
        $dumpfile("tx_tb.vcd");
        $dumpvars(0, tx_tb);
    end

endmodule