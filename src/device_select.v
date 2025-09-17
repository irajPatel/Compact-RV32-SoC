module device_select (
    input [31:0] addr,
    output s0_sel_mem,
    output s1_sel_gpio,
    output s2_sel_uart
    // output s3_sel,
    // output s4_sel,
    // output s5_sel,
    // output s6_sel,
    // output s7_sel,
    // output s8_sel
);


  wire mem_space = (addr[31:28] == 4'h0);
  wire gpio_space = (addr[31:28] == 4'h4);
  wire uart_space = (addr[31:28] == 4'h5);

  assign s0_sel_mem  = mem_space;
  assign s1_sel_gpio = gpio_space;
  assign s2_sel_uart = uart_space;

endmodule
