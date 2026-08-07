`ifndef TX_NODE_SEQUENCE_SV
`define TX_NODE_SEQUENCE_SV

// Sequence model with memory

// One sequence belongs to one single node
// LOAD+ then a single COMMIT/ABORT that releases the whole open set, repeated. 
// Cross-node conflicts arise naturally because every node's loads are weighted into CONFLICT_REGION by the item's dist.

import tx_pkg::*;

class tx_node_sequence extends uvm_sequence #(tx_transaction);
    `uvm_object_utils(tx_node_sequence)

    // Configurations for the entire sequence
    int unsigned node_id = 0; // plain index 0..NUM_NODES-1; one-hot only on the item
    int unsigned num_txns = 10; // number of transactions this node runs
    int unsigned commit_pct = 70; // % of closes that are COMMIT vs self-ABORT
    int unsigned write_pct = 60; // % of released slots classified as write vs read-only
    int unsigned bubble_pct = 20; // % chance of an idle item between transactions

    // We can only load as much as we can release at once
    localparam int MAX_LOADS = RELEASE_SET_DEPTH;

    // data structure holding this node's currently-checked-out addresses
    // associative array (conceptually a set)
    // O(1) membership and set semantics (uniqueness free)
    bit open_set [ bit[ADDR_W-1:0] ];

    function new(string name = "tx_node_sequence");
        super.new(name);
    endfunction

    // convert plain index node_id to one-hot encoding
    function logic [NUM_NODES-1:0] onehot();
        return (logic [NUM_NODES-1:0]'(1) << node_id);
    endfunction

    virtual task body();
        assert (node_id < NUM_NODES) else `uvm_fatal("SEQ", "node_id out of range");

        for (int t = 0; t < num_txns; t++) begin
            // determine the number of loads and send them
            int n_loads = $urandom_range(1, MAX_LOADS);
            for (int l = 0; l < n_loads; l++) send_load();

            // send in closer
            send_close();

            // send in a bubble if we defined it
            if ($urandom_range(1, 100) <= bubble_pct) send_bubble();
        end
    endtask

    // LOAD
    task send_load();
        tx_transaction req;

        req = tx_transaction::type_id::create("load");
        start_item(req);
            req.tx_type = CMD_LOAD;
            req.req_node = onehot();
            req.release_is_write = '0; // logic defaults to X; clear for clean waves
            req.release_is_read  = '0;
            req.release_addr = '0;
            // randomize load address
            assert (req.randomize(load_addr))
                else `uvm_fatal("SEQ", "load_addr randomize failed");
        finish_item(req);

        open_set[req.load_addr] = 1;          // now checked out for this node
    endtask

    // COMMIT/ABORT
    task send_close();
        tx_transaction req;
        bit [ADDR_W-1:0] pool [$];
        int k; // release set size

        assert (open_set.size() > 0) // lifecycle: never close an empty node
            else `uvm_fatal("SEQ", "close with no open loads");

        foreach (open_set[a]) pool.push_back(a); // create pool
        pool.shuffle(); // randomize slot ordering
        k = pool.size(); // faithful: Release_Set is the whole touched set
        assert (k <= MAX_LOADS)
            else `uvm_fatal("SEQ", "open set exceeds MAX_LOADS"); // cannot have loaded more than there exists

        req = tx_transaction::type_id::create("close");
        start_item(req);
            // determine req type
            req.tx_type = ($urandom_range(1,100) <= commit_pct) ?  CMD_TX_COMMIT : CMD_TX_ABORT;
            req.req_node = onehot();
            req.release_is_write = '0;
            req.release_is_read  = '0;
            req.release_addr = '0;

            // write data for every slot; masked away where not a write
            assert (req.randomize(release_data))
                else `uvm_fatal("SEQ", "release_data randomize failed");

            // assign read and write entries
            for (int i = 0; i < k; i++) begin
                req.release_addr[i] = pool[i];
                if ($urandom_range(1,100) <= write_pct) req.release_is_write[i] = 1'b1;
                else req.release_is_read [i] = 1'b1;
            end

            // assertion constraints
            assert ($onehot(req.req_node))
                else `uvm_fatal("SEQ", "req_node not one-hot on a real command");
            assert ((req.release_is_write & req.release_is_read) == '0)
                else `uvm_fatal("SEQ", "write/read overlap"); // no overlap
            assert ($countones(req.release_is_write | req.release_is_read) <= MAX_RELEASE_SET)
                else `uvm_fatal("SEQ", "release set exceeds credit cap");
            for (int i = 0; i < k; i++)
                assert (open_set.exists(req.release_addr[i]))
                    else `uvm_fatal("SEQ", "releasing un-loaded address");
        finish_item(req);

        open_set.delete();
    endtask

    // BUBBLE 
    task send_bubble();
        tx_transaction req;
        req = tx_transaction::type_id::create("bubble");
        start_item(req);
            req.req_node = '0; // no node asserting == bubble
            req.release_is_write = '0;
            req.release_is_read  = '0;
            req.release_addr = '0;
        finish_item(req);
    endtask

endclass

`endif