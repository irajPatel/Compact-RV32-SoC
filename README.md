# picoSoC_v3

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

---

If you'd like, I can:
- Tailor the README with exact commands for your toolchain (which riscv gcc version you use).
- Add example memory maps and addresses used by `device_select` and peripherals.
- Generate the Corsair YAML snippets used to produce the UART/GPIO register maps.

Tell me which addition you'd prefer next and I'll update the README accordingly.
