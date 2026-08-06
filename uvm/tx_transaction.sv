`ifndef TX_TRANSACTION_SV
`define TX_TRANSACTION_SV

import tx_pkg::*;


class tx_transaction extends uvm_sequence_item;
    `uvm_object_utils(tx_transaction)

    tx_type_t tx_type; // type needs restrictions in sequence level (can't commit/abort before load)
    logic [NUM_NODES-1:0] req_node;
    logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr; // depends on history
    logic [RELEASE_SET_DEPTH-1:0] release_is_write; // sequence set
    logic [RELEASE_SET_DEPTH-1:0] release_is_read; // sequence set

    rand logic [ADDR_W-1:0] load_addr;
    rand logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data; // fill arbitrarily, validity will mask
    // constraint load_addr_uniform { load_addr inside {[0:MEM_DEPTH-1]}; }
    // Apply weight to force contention and aborts
    constraint load_addr_hot {
        load_addr dist {
            [0 : CONFLICT_REGION-1] := 80,
            [CONFLICT_REGION : MEM_DEPTH-1] := 20
        };
    }
    // load_addr must be within memory size

    // constructor
    function new(string name = "tx_transaction");
        super.new(name);
    endfunction

    // custom functions
    // Copy a tx_transaction object
    function void do_copy(uvm_object rhs);
        tx_transaction r;
        if (!$cast(r, rhs)) `uvm_fatal("do_copy", "cast failed")
        super.do_copy(rhs);

        // assign fields of casted result
        tx_type = r.tx_type;
        req_node = r.req_node;
        release_is_write = r.release_is_write;
        release_is_read  = r.release_is_read;
        release_addr = r.release_addr; // copy all slots; dead ones are harmless
        release_data = r.release_data;
    endfunction

    // custom do_compare, compares between 
    function bit do_compare(uvm_object rhs, uvm_comparer comparer);
        tx_transaction r;
        bit ok = 1;
        if (!$cast(r, rhs)) return 0;
        
        // invoke default parent compare for the whole object
        ok &= super.do_compare(rhs, comparer);
        // invoke default field compares for 1D fields
        ok &= comparer.compare_field_int("tx_type", tx_type, r.tx_type, $bits(tx_type));
        ok &= comparer.compare_field_int("req_node", req_node, r.req_node, $bits(req_node));
        ok &= comparer.compare_field("is_write", release_is_write, r.release_is_write, $bits(release_is_write));
        ok &= comparer.compare_field("is_read",  release_is_read,  r.release_is_read,  $bits(release_is_read));
        // iterative compare for 2D fields
        for (int i = 0; i < RELEASE_SET_DEPTH; i++) begin
            if (release_is_write[i] || release_is_read[i])
                ok &= comparer.compare_field($sformatf("addr[%0d]", i),
                          release_addr[i], r.release_addr[i], ADDR_W);
            if (release_is_write[i])   // only writes carry meaningful data
                ok &= comparer.compare_field($sformatf("data[%0d]", i),
                          release_data[i], r.release_data[i], DATA_W);
        end
        return ok;
    endfunction

    // custom do_print 
    function void do_print(uvm_printer printer);
        super.do_print(printer);
        printer.print_string("tx_type",  tx_type.name());
        printer.print_field ("req_node", req_node, $bits(req_node), UVM_BIN);
        printer.print_field("is_write", release_is_write, $bits(release_is_write), UVM_BIN);
        printer.print_field("is_read",  release_is_read,  $bits(release_is_read),  UVM_BIN);
        for (int i = 0; i < RELEASE_SET_DEPTH; i++) begin
            if (release_is_write[i] || release_is_read[i]) begin
                printer.print_field($sformatf("addr[%0d]", i), release_addr[i], ADDR_W, UVM_HEX);
                if (release_is_write[i])
                    printer.print_field($sformatf("data[%0d]", i), release_data[i], DATA_W, UVM_HEX);
            end
        end
    endfunction

endclass

`endif
