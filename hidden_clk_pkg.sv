package hidden_clk_pkg;

typedef struct packed {
    logic               clk;
} hidden_clk_t;

function void connect(inout hidden_clk_t hidden_clk, input logic set);

    hidden_clk_t global_hidden_clk;

    if (set)
        global_hidden_clk = hidden_clk;

    hidden_clk = global_hidden_clk;
endfunction

endpackage