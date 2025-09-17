// Created with Corsair v1.0.4

module regs_uart #(
    parameter ADDR_W = 32,
    parameter DATA_W = 32,
    parameter STRB_W = DATA_W / 8
)(
    // System
    input clk,
    input rst,
    // U_DATA.DATA
    output [7:0] csr_u_data_data_out,

    // U_STAT.READY
    input  csr_u_stat_ready_in,
    // U_STAT.TX_DONE
    input  csr_u_stat_tx_done_in,

    // U_CTRL.START
    output  csr_u_ctrl_start_out,

    // Local Bus
    input  [ADDR_W-1:0] waddr,
    input  [DATA_W-1:0] wdata,
    input               wen,
    input  [STRB_W-1:0] wstrb,
    output              wready,
    input  [ADDR_W-1:0] raddr,
    input               ren,
    output [DATA_W-1:0] rdata,
    output              rvalid
);
//------------------------------------------------------------------------------
// CSR:
// [0x0] - U_DATA - UART Data register
//------------------------------------------------------------------------------
wire [31:0] csr_u_data_rdata;
assign csr_u_data_rdata[31:8] = 24'h0;

wire csr_u_data_wen;
assign csr_u_data_wen = wen && (waddr == 32'h0);

wire csr_u_data_ren;
assign csr_u_data_ren = ren && (raddr == 32'h0);
reg csr_u_data_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_u_data_ren_ff <= 1'b0;
    end else begin
        csr_u_data_ren_ff <= csr_u_data_ren;
    end
end
//---------------------
// Bit field:
// U_DATA[7:0] - DATA - Data To Send Via UART TX
// access: rw, hardware: o
//---------------------
reg [7:0] csr_u_data_data_ff;

assign csr_u_data_rdata[7:0] = csr_u_data_data_ff;

assign csr_u_data_data_out = csr_u_data_data_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_u_data_data_ff <= 8'h0;
    end else  begin
     if (csr_u_data_wen) begin
            if (wstrb[0]) begin
                csr_u_data_data_ff[7:0] <= wdata[7:0];
            end
        end else begin
            csr_u_data_data_ff <= csr_u_data_data_ff;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x4] - U_STAT - UART Status register
//------------------------------------------------------------------------------
wire [31:0] csr_u_stat_rdata;
assign csr_u_stat_rdata[4:0] = 5'h0;
assign csr_u_stat_rdata[12:6] = 7'h0;
assign csr_u_stat_rdata[31:14] = 18'h0;

wire csr_u_stat_wen;
assign csr_u_stat_wen = wen && (waddr == 32'h4);

wire csr_u_stat_ren;
assign csr_u_stat_ren = ren && (raddr == 32'h4);
reg csr_u_stat_ren_ff;
always @(posedge clk) begin
    if (rst) begin
        csr_u_stat_ren_ff <= 1'b0;
    end else begin
        csr_u_stat_ren_ff <= csr_u_stat_ren;
    end
end
//---------------------
// Bit field:
// U_STAT[5] - READY - UART is Ready
// access: ro, hardware: i
//---------------------
reg  csr_u_stat_ready_ff;

assign csr_u_stat_rdata[5] = csr_u_stat_ready_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_u_stat_ready_ff <= 1'b1;
    end else  begin
              begin            csr_u_stat_ready_ff <= csr_u_stat_ready_in;
        end
    end
end


//---------------------
// Bit field:
// U_STAT[13] - TX_DONE - Done Transmitting The Last Char (8-bit)
// access: roc, hardware: i
//---------------------
reg  csr_u_stat_tx_done_ff;

assign csr_u_stat_rdata[13] = csr_u_stat_tx_done_ff;


always @(posedge clk) begin
    if (rst) begin
        csr_u_stat_tx_done_ff <= 1'b0;
    end else  begin
          if (csr_u_stat_ren && !csr_u_stat_ren_ff) begin
            csr_u_stat_tx_done_ff <= 1'b0;
        end else            begin            csr_u_stat_tx_done_ff <= csr_u_stat_tx_done_in;
        end
    end
end


//------------------------------------------------------------------------------
// CSR:
// [0x8] - U_CTRL - UART Control register
//------------------------------------------------------------------------------
wire [31:0] csr_u_ctrl_rdata;
assign csr_u_ctrl_rdata[8:0] = 9'h0;
assign csr_u_ctrl_rdata[31:10] = 22'h0;

wire csr_u_ctrl_wen;
assign csr_u_ctrl_wen = wen && (waddr == 32'h8);

//---------------------
// Bit field:
// U_CTRL[9] - START - TX Begin Signal, Valid For Only One Cycle
// access: wosc, hardware: o
//---------------------
reg  csr_u_ctrl_start_ff;

assign csr_u_ctrl_rdata[9] = 1'b0;

assign csr_u_ctrl_start_out = csr_u_ctrl_start_ff;

always @(posedge clk) begin
    if (rst) begin
        csr_u_ctrl_start_ff <= 1'b0;
    end else  begin
     if (csr_u_ctrl_wen) begin
            if (wstrb[1]) begin
                csr_u_ctrl_start_ff <= wdata[9];
            end
        end else begin
            csr_u_ctrl_start_ff <= 1'b0;
        end
    end
end


//------------------------------------------------------------------------------
// Write ready
//------------------------------------------------------------------------------
assign wready = 1'b1;

//------------------------------------------------------------------------------
// Read address decoder
//------------------------------------------------------------------------------
reg [31:0] rdata_ff;
always @(posedge clk) begin
    if (rst) begin
        rdata_ff <= 32'h0;
    end else if (ren) begin
        case (raddr)
            32'h0: rdata_ff <= csr_u_data_rdata;
            32'h4: rdata_ff <= csr_u_stat_rdata;
            32'h8: rdata_ff <= csr_u_ctrl_rdata;
            default: rdata_ff <= 32'h0;
        endcase
    end else begin
        rdata_ff <= 32'h0;
    end
end
assign rdata = rdata_ff;

//------------------------------------------------------------------------------
// Read data valid
//------------------------------------------------------------------------------
reg rvalid_ff;
always @(posedge clk) begin
    if (rst) begin
        rvalid_ff <= 1'b0;
    end else if (ren && rvalid) begin
        rvalid_ff <= 1'b0;
    end else if (ren) begin
        rvalid_ff <= 1'b1;
    end
end

assign rvalid = rvalid_ff;

endmodule