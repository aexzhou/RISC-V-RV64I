// https://github.com/chiggs/hidden_wires

package hidden_wires_pkg;

typedef struct packed {
    logic [63:0]        address;
    logic [31:0]        data;
    logic               enable;
} hidden_wires_t;


// Take advantage of the fact that local variables in functions are
// static by default to perform inter-module communication
function void connect(inout hidden_wires_t wires, input logic set);

    hidden_wires_t global_wires;

    if (set)
        global_wires = wires;

    wires = global_wires;
endfunction

endpackage


package hidden_clk_pkg;

typedef struct packed {
    logic               clk;
} hidden_clk_t;

function void connect(inout hidden_clk_t clk, input logic set);

    hidden_clk_t global_clk;

    if (set)
        global_clk = clk;

    clk = global_clk;
endfunction

endpackage
