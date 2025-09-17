# Compact-RV32-SoC

## Introduction

`picoSoC_v3` is a lightweight RISC-V System-on-Chip (SoC) project intended for learning, experimentation and small FPGA/ASIC prototyping. It builds on a compact RV32 core (FemtoRV32 / PicoRV32 family) and integrates simple peripherals (GPIO, UART) to form a minimal but useful development platform.

This repository contains RTL sources, a small firmware image, testbenches, and helper scripts to simulate and run the SoC.

## Features

- RV32I-based CPU core with simple instruction fetch/decode/execute pipeline.
- Memory module and flash/ROM initialization from a firmware hex file.
- IP cores: GPIO and UART (register-mapped peripherals).
- Simple bus/device-selection logic for memory-mapped IO.
- Testbench and simulation flow using Icarus Verilog and GTKWave.

## Peripherals & IP Cores

### GPIO
- Behaves as a memory-mapped peripheral: writes to the GPIO registers update output bits (e.g. LEDs), reads return current GPIO state.
- Simple interface: write mask / data registers mapped into the processor address space.
- Integrated in `top.v` via the `device_select` logic: certain address ranges are routed to the GPIO unit.

### UART
- Basic UART IP core for byte-oriented TX/RX.
- Exposes status and data registers through a Corsair-generated register map (see below).
- Interrupt/handshake lines (if present) are connected to the SoC core or left for firmware polling depending on the design.

### Corsair Usage (IP Generation)
- Corsair is used to rapidly define and generate peripheral IP cores and register maps from concise YAML/JSON descriptions.
- Basics:
  - Define peripheral registers, fields and access types in a Corsair project file.
  - Corsair generates Verilog wrappers and C header files describing register offsets and bitfields.
- How we used it:
  - The UART and GPIO register maps were created using Corsair descriptions.
  - Corsair generated the peripheral skeletons and header files (`*_regs.h`) used by the firmware to access the peripherals.
  - This ensures consistent register offsets between RTL and firmware.

  Detailed device mapping and register maps
  ---------------------------------------

  - Address decoding (device_select): the SoC uses the top 4 bits of a 32-bit bus address to route accesses.
    - `0x0_______` : main memory (ROM/RAM)
    - `0x4_______` : GPIO peripheral (base address `0x40000000`)
    - `0x5_______` : UART peripheral (base address `0x50000000`)

  - The `device_select` logic (in `src/device_select.v`) is intentionally minimal: it compares the high nibble of `mem_addr` and asserts one-hot select signals:

  ```verilog
  // simplified excerpt from `device_select.v`
  wire mem_space  = (addr[31:28] == 4'h0);
  wire gpio_space = (addr[31:28] == 4'h4);
  wire uart_space = (addr[31:28] == 4'h5);

  assign s0_sel_mem  = mem_space;
  assign s1_sel_gpio = gpio_space;
  assign s2_sel_uart = uart_space;
  ```

  - How reads/writes are routed in `top.v`:
    - The processor drives `mem_addr`, `mem_wdata`, `mem_wmask` and `mem_rstrb`.
    - `device_select` asserts `s0_sel_mem`, `s1_sel_gpio`, or `s2_sel_uart` depending on the `mem_addr` high nibble.
    - The `top.v` module multiplexes read data back to the processor using these signals. For example:

  ```verilog
  // excerpt from `top.v`
  case ({s2_sel_uart, s1_sel_gpio, s0_sel_mem})
    3'b001: processor_rdata = mem_rdata;    // memory
    3'b010: processor_rdata = rdata_gpio;    // gpio
    3'b100: processor_rdata = rdata_uart;    // uart
  endcase
  ```

  Local-bus interface
  -------------------

  - Both `gpio_ip` and `uart_ip` implement the same simple local-bus interface used by `top.v`:
    - `waddr[31:0]` / `raddr[31:0]` : local 32-bit addresses (top module supplies `{4'h0, mem_addr[27:0]}`)
    - `wdata[31:0]`, `wen`, `wstrb[3:0]` : write data + byte-lane strobes
    - `rdata[31:0]`, `ren`, `rvalid` : read data + read handshake
    - `wready` : write accepted

  - Note: `top.v` constructs a local-bus address by concatenating `4'h0` to `mem_addr[27:0]` before feeding to peripherals. This effectively leaves room for future sub-decoding inside each peripheral.

  GPIO register map (from `firmware/inc/gpio_regs.h`)
  -------------------------------------------------

  - Base address: `0x40000000` (as defined by `GCSR_BASE_ADDR`)
  - Registers (offsets from base):
    - `0x00` - `GPIO_0` (32-bit read/write) : DATA — drives LED outputs

  Register details (GPIO_0)
  - Width: 32 bits
  - Access: read/write
  - Behavior: writes update the `csr_gpio_0_data_ff` register in the IP; the IP exposes `csr_gpio_0_data_out` which is connected to `leds` in `top.v`.

  Example access sequence (software):
  1. Write 32-bit value to address `0x40000000` using regular store instruction.
  2. `top.v` asserts `s1_sel_gpio` and `wen` to `gpio_ip`.
  3. `gpio_ip` latches bytes according to `wstrb` and updates `csr_gpio_0_data_ff`.
  4. The `csr_gpio_0_data_out` output drives `leds` on the top-level.

  UART register map (from `firmware/inc/uart_regs.h`)
  -------------------------------------------------

  - Base address: `0x50000000` (as defined by `UCSR_BASE_ADDR`)
  - Registers (offsets from base):
    - `0x00` - `U_DATA` (32-bit write/read) : DATA (8-bit payload in LSB)
    - `0x04` - `U_STAT` (32-bit read) : status bits (READY at bit 5, TX_DONE at bit 13)
    - `0x08` - `U_CTRL` (32-bit write-only) : control bits (START at bit 9)

  Register behavior and connection to `uart_ip`
  - Writing to `U_DATA` places the 8-bit character into `csr_u_data_data_out` inside `uart_ip`.
  - Writing to `U_CTRL.START` (bit 9) issues a one-cycle start pulse `csr_u_ctrl_start_out` that the UART transmitter uses to sample data and begin transmission.
  - `uart_ip` exposes a `csr_u_stat_ready_in` signal fed by the transmitter `o_ready` to indicate the transmitter can accept new data. The `regs_uart` interface maps that into `U_STAT.READY`.
  - When a character is sent, `U_STAT.TX_DONE` is asserted for one cycle (`csr_u_stat_tx_done_in`) to allow firmware to detect transmission completion.

  How software typically drives the UART
  1. Poll `U_STAT.READY` (bit 5) at `0x50000004` until set.
  2. Write character into `0x50000000` (U_DATA) with the payload in LSB.
  3. Write to `0x50000008` (U_CTRL) to toggle START (or the `regs_uart` wrapper may assert START automatically when data is written, depending on the auto-generated glue logic).

  Consistency via Corsair
  -----------------------
  - Corsair generated the `gpio_regs.h` and `uart_regs.h` headers; these headers define `BASE_ADDR` constants and register offsets, ensuring firmware writes go to exactly the addresses expected by RTL. Keep the header files synchronized with RTL if you edit register layouts.

## Firmware & Linker Script

- Firmware is a small C program compiled to an ELF and converted to a hex/ram image for the simulation ROM.
- The linker script (e.g., `sections.lds` or `linker.ld`) defines memory regions and placement of text/data/rodata and peripheral memory mappings.
- The linker script ensures interrupt vectors, .text and .data sections land at addresses expected by the boot ROM and `top.v` memory map.
- For memory-mapped peripherals, the linker script may reserve regions or place device vectors appropriately so the firmware accesses devices at correct addresses.

## Top-Level Design (`top.v`)

- `top.v` instantiates the memory, CPU core and device select logic and wires up GPIO and UART.
- The module provides a clean separation of memory and peripheral spaces:
  - Memory accesses are routed to the `Memory` block for instruction fetches and data.
  - Memory-mapped IO addresses are detected by `device_select` (or `dv_sel`) which asserts select signals like `s0_sel_mem` and `s1_sel_leds`.
- The `leds` register in `top.v` is written by the SoC via memory writes to the mapped GPIO address range.

## Simulation & Dependencies

### Tools
- Icarus Verilog (`iverilog`) — compile and simulate Verilog testbenches.
- GTKWave (`gtkwave`) — view VCD waveform dumps for debugging.
- GNU toolchain (gcc, avr/gcc or riscv toolchain) — compile the firmware (`riscv32-unknown-elf-gcc` if targeting RISC-V).
- Python (3.x) — helper scripts for converting ELF to hex (`makehex.py`) or utilities.

### Recommended Packages (Ubuntu/Debian)
```bash
sudo apt update
sudo apt install iverilog gtkwave build-essential python3 python3-pip make
# If building RISC-V firmware, install the RISC-V toolchain (or use a prebuilt toolchain)
# Example (apt may not have it up-to-date):
# sudo apt install gcc-riscv64-unknown-elf
```

### Simulate
- To compile the design and the testbench:
```bash
cd /home/iraj/LearnSoC/picoSoC_v3
iverilog -g2012 src/*.v tb_processor.v -o prog
vvp prog
```
- Waveforms (if the testbench produces a VCD file):
```bash
gtkwave wave.vcd
```

Note: If simulation fails due to elaboration errors, check the RTL for use-before-declare issues or syntax errors (Icarus shows file/line where the error occurred).

Simulation workflow (detailed)
------------------------------

- Run the testbench and generate a VCD waveform using Icarus Verilog:

```bash
cd /home/iraj/LearnSoC/picoSoC_v3
iverilog -g2012 src/*.v tb_processor.v -o prog
vvp prog       # the TB should create `wave.vcd`
gtkwave wave.vcd
```

- If `tb_processor.v` does not create a VCD by default, add a dump section to the testbench (example):

```verilog
initial begin
  $dumpfile("wave.vcd");
  $dumpvars(0, tb_processor);
  #10000 $finish;
end
```

- Example waveform snapshot (add a real image after running sim):

![Waveform example](doc/images/waveform_example.png)

Tips for debugging with waves
- Focus on `mem_addr`, `mem_wdata`, `mem_wmask`/`mem_wstrb`, `mem_rstrb`, `processor_rdata`, and peripheral `rdata`/`wready`/`rvalid` signals.
- Watch `s0_sel_mem`, `s1_sel_gpio`, `s2_sel_uart` to verify proper address decoding.
- For UART debugging, inspect `U_DATA`, `U_CTRL`, `U_STAT` register writes/reads and `o_uart_tx` transitions.

## Usage Guide: Build & Test

1. Build firmware
   - Edit `firmware/main.c` as needed.
   - Use the provided `Makefile` in the `firmware/` directory. Typical flow:

```bash
cd firmware
make
```

2. Generate hex image

```bash
python3 makehex.py firmware.elf > firmware.hex
# or the project Makefile already creates firmware.hex
```

3. Run simulation

```bash
cd /home/iraj/LearnSoC/picoSoC_v3
iverilog -g2012 src/*.v tb_processor.v -o prog
vvp prog
gtkwave wave.vcd
```

## Future Development

- I2C and SPI peripheral IPs are under development and will be added as additional Corsair modules.
- Plan to target FPGA boards for hardware validation.
- Long-term: explore ASIC flow using OpenLane and Qflow to generate GDS and run place-and-route for tapeout-ready flows.

## Acknowledgments
- FemtoRV32 / PicoRV32 authors for compact RISC-V cores that inspired this project.
- Corsair project for making register and IP generation easier.


