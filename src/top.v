module top (
    input clk,
    input reset_n,
    output  [4:0] leds,
    output uart_tx
);

   parameter MEM_FILE = "/home/iraj/LearnSoC/picoSoC_v3/firmware/firmware.hex";
   
   wire mem_rstrb;
   wire mem_instr;
   wire mem_ready;
   wire [31:0] mem_addr;
   wire [31:0] mem_wdata;
   wire [3:0] mem_wstrb;
   wire [31:0] mem_rdata, rdata_gpio, rdata_uart;

   wire s0_sel_mem;
   wire s1_sel_gpio;
   wire s2_sel_uart;

//    wire [31:0] processor_rdata = (s1_sel_leds)? {{27{1'b0}},leds} : mem_rdata;
   reg [31:0] processor_rdata;

   always @(*) begin
        processor_rdata = 32'h0;
        case ({s2_sel_uart, s1_sel_gpio, s0_sel_mem})
            3'b001: processor_rdata = mem_rdata;
            3'b010: processor_rdata = rdata_gpio;
            3'b100: processor_rdata = rdata_uart;
        endcase
   end


  Memory #(
      .MEM_FILE(MEM_FILE),
      .SIZE(1024)
  ) D_mem_unit (
      .clk(clk),
      .mem_addr(mem_addr),
      .mem_rdata(mem_rdata),
      .mem_rstrb(s0_sel_mem & mem_rstrb),
      .mem_wdata(mem_wdata),
      .mem_wmask({4{s0_sel_mem}} & mem_wstrb)
  );

  FemtoRV32  processor (
      .clk      (clk),
      .reset   (reset_n),
      .mem_rstrb(mem_rstrb),
      .mem_rbusy(1'b0),
      .mem_wbusy(1'b0),
      .mem_addr (mem_addr),
      .mem_wdata(mem_wdata),
      .mem_wmask(mem_wstrb),
      .mem_rdata(processor_rdata)
  );

   gpio_ip gpio_unit(
    // System
    .clk(clk),
    .rst(!reset_n),
    // GPIO_0.DATA
    .csr_gpio_0_data_out(leds),

    // Local Bus
    .waddr({4'h0, mem_addr[27:0]}),
    .wdata(mem_wdata),
    .wen(s1_sel_gpio & (|mem_wstrb)),
    .wstrb(mem_wstrb),
    .wready(),
    .raddr({4'h0, mem_addr[27:0]}),
    .ren(s1_sel_gpio & mem_rstrb),
    .rdata(rdata_gpio),
    .rvalid()
    );

   uart_ip uart_unit(

    // System
    .clk(clk),
    .rst(!reset_n),
    // Local Bus
    .waddr({4'h0, mem_addr[27:0]}),
    .wdata(mem_wdata),
    .wen(s2_sel_uart & (|mem_wstrb)),
    .wstrb(mem_wstrb),
    .wready(),
    .raddr({4'h0, mem_addr[27:0]}),
    .ren(s2_sel_uart & mem_rstrb),
    .rdata(rdata_uart),
    .rvalid(),
    // uart tx
    .o_uart_tx(uart_tx)


    );

  device_select dv_sel(
    .addr(mem_addr),
    .s0_sel_mem(s0_sel_mem),
    .s1_sel_gpio(s1_sel_gpio),
    .s2_sel_uart(s2_sel_uart)
    );

// always @(posedge clk or negedge reset_n) begin
//     if (!reset_n) begin
//         leds <= 'h0;
//     end 
//     else begin
//         if (s1_sel_leds &  mem_wstrb[0]) begin
//             leds <= mem_wdata[4:0];
//         end
//     end
// end

endmodule
