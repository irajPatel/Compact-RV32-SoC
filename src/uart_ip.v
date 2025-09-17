module uart_ip #(
    parameter ADDR_W = 32,
    parameter DATA_W = 32,
    parameter STRB_W = DATA_W / 8
)(

    // System
    input clk,
    input rst,

    // Local Bus
    input  [ADDR_W-1:0] waddr,
    input  [DATA_W-1:0] wdata,
    input               wen,
    input  [STRB_W-1:0] wstrb,
    output              wready,
    input  [ADDR_W-1:0] raddr,
    input               ren,
    output [DATA_W-1:0] rdata,
    output              rvalid,

    // uart tx

    output  o_uart_tx


);

    reg o_ready_reg;

    

    wire [7:0] csr_u_data_data_out;
    wire       o_ready;
    wire       csr_u_ctrl_start_out;


    regs_uart  uart_regs_interface_u0(

        .clk(clk),
        .rst(rst),
        .csr_u_data_data_out(csr_u_data_data_out),
        .csr_u_stat_ready_in(o_ready),
        .csr_u_stat_tx_done_in( !o_ready_reg & o_ready  ),
        .csr_u_ctrl_start_out(csr_u_ctrl_start_out),
        .waddr(waddr),
        .wdata(wdata),
        .wen(wen),
        .wstrb(wstrb),
        .wready(wready),
        .raddr(raddr),
        .ren(ren),
        .rdata(rdata),
        .rvalid(rvalid)

    );


    uart_tx uart_transmitter_u1 (

        .i_clk(clk),
        .i_rst(rst),
        .i_data(csr_u_data_data_out),
        .i_valid(csr_u_ctrl_start_out),
        .o_ready(o_ready),
        .o_uart_tx(o_uart_tx)

    );


    always @(posedge clk) begin
        o_ready_reg <= o_ready;
    end



endmodule