`ifndef TX_SEQUENCE_SV
`define TX_SEQUENCE_SV

import tx_pkg::*;

class tx_sequence extends uvm_sequence #(tx_transaction);
    `uvm_object_utils(tx_sequence)

    // randomized once per transaction sequence
    rand logic [X_COORD_W-1:0] dst_x;
    rand logic [Y_COORD_W-1:0] dst_y;
    rand logic [X_COORD_W-1:0] src_x;
    rand logic [Y_COORD_W-1:0] src_y;
    rand logic [PACKET_ID_W-1:0] pkt_id;
    rand int unsigned N;

    constraint dst_x_bounds { dst_x inside {[0:MESH_DIM_X-1]}; }
    constraint dst_y_bounds { dst_y inside {[0:MESH_DIM_Y-1]}; }
    constraint src_x_bounds { src_x inside {[0:MESH_DIM_X-1]}; }
    constraint src_y_bounds { src_y inside {[0:MESH_DIM_Y-1]}; }
    constraint pkt_id_bounds { pkt_id inside {[0:(2**PACKET_ID_W)-1]}; }
    constraint body_count { N inside {[1:4]}; }

    function new(string name = "flit_sequence");
        super.new(name);
    endfunction

    task body();
        flit_transaction t;

        // HEAD
        t = flit_transaction::type_id::create("t");
        start_item(t);
        t.flit_type = HEAD;
        t.dst_x = dst_x;
        t.dst_y = dst_y;
        t.src_x = src_x;
        t.src_y = src_y;
        t.pkt_id = pkt_id;
        t.vc_id = '0;
        assert(t.randomize());
        finish_item(t);

        // BODY
        repeat(N) begin
            t = flit_transaction::type_id::create("t");
            start_item(t);
            t.flit_type = BODY;
            t.pkt_id = pkt_id;
            t.vc_id = '0;
            assert(t.randomize());
            finish_item(t);
        end

        // TAIL
        t = flit_transaction::type_id::create("t");
        start_item(t);
        t.flit_type = TAIL;
        t.pkt_id = pkt_id;
        t.vc_id = '0;
        assert(t.randomize());
        finish_item(t);
    endtask

endclass

`endif