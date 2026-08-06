package tx_pkg;
    // Top-level parameters
    localparam int DATA_W = 64;
    localparam int ADDR_W = 64;
    localparam int NUM_NODES = 4;
    localparam int RELEASE_SET_DEPTH = 16;
    localparam int CXL_TABLE_DEPTH = 32;
    localparam int BUF_DEPTH = 16;
    localparam int MEM_DEPTH = 1024;
    localparam int MEM_LATENCY = 1;
    localparam int CONFLICT_REGION = 16; // for creating conflict

    // Command type encoding
    typedef enum logic [1:0] {
        CMD_LOAD = 2'b00,
        CMD_TX_ABORT  = 2'b01,
        CMD_TX_COMMIT = 2'b10
    } tx_type_t;

    // Load command structure
    typedef struct packed {
        tx_type_t tx_type;
        logic [NUM_NODES-1:0] req_node;
        logic [ADDR_W-1:0] load_addr; // dest x
    } tx_load_t;

    // Abort command structure
    typedef struct packed {
        tx_type_t tx_type;
        logic [NUM_NODES-1:0] req_node;
        // logic [RELEASE_SET_DEPTH-1:0] release_is_write;
        logic [RELEASE_SET_DEPTH-1:0] release_is_read; // only one release set needed
        logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr;
    } tx_abort_t;

    // Commit command structure
    typedef struct packed {
        tx_type_t tx_type;
        logic [NUM_NODES-1:0] req_node;
        logic [RELEASE_SET_DEPTH-1:0] release_is_write;
        logic [RELEASE_SET_DEPTH-1:0] release_is_read;
        logic [RELEASE_SET_DEPTH-1:0][ADDR_W-1:0] release_addr;
        logic [RELEASE_SET_DEPTH-1:0][DATA_W-1:0] release_data;
    } tx_commit_t;

endpackage
