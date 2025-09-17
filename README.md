
# 🖥️ Compact-RV32-SoC

**Tiny RISC-V SoC for learning, experimentation, and FPGA prototyping!**
This SoC is built around a **FemtoRV32 CPU**, with **memory**, **GPIO**, and **UART** peripherals. The design is simulation-ready using **Icarus Verilog** and runs simple firmware compiled via **GCC**.

---

## 📂 File Hierarchy & Purpose

| Folder / File           | Description                                                                     |
| ----------------------- | ------------------------------------------------------------------------------- |
| `Compact-RV32-SoC/`           | Current main project                                                            |
| `src/top.v`             | Top-level SoC module connecting CPU, memory, peripherals, and address decoder   |
| `src/device_select.v`   | Simple address decoder (`addr[31:28]`) asserting selects for memory, GPIO, UART |
| `src/Memory.v`          | Simple synchronous memory/ROM for firmware (`0x00000000`)                       |
| `src/femtorv32_quark.v` | FemtoRV32 CPU core (Quark variant) handling fetch, decode, ALU, load/store, CSR |
| `src/gpio_ip.v`         | Corsair-style GPIO peripheral wrapper (byte-strobe writes, read handshake)      |
| `src/uart_ip.v`         | UART peripheral wrapper (`U_DATA`, `U_STAT`, `U_CTRL`) and transmitter logic    |
| `firmware/main.c`       | Example firmware demonstrating LED blink + UART prints                          |
| `firmware/makehex.py`   | Converts ELF → hex for simulation ROM                                           |
| `firmware/Makefile`     | Build firmware using GCC                                                        |
| `firmware/sections.lds` | Linker script controlling `.text` section placement in ROM                      |
| `firmware/inc/`         | Corsair-generated header files (`gpio_regs.h`, `uart_regs.h`)                   |
| `tb_processor.v`        | Top-level testbench producing VCD waveforms                                     |
| `prog` / `prog.exe`     | Compiled simulation binary (`iverilog` output)                                  |

---

## 🚀 Features

* RV32I CPU core (FemtoRV32/Quark variant)
* Synchronous memory + ROM initialization from firmware hex
* IP cores: GPIO & UART (Corsair-generated register-mapped peripherals)
* Simple **bus / device selection logic** using high nibble of 32-bit addresses
* Simulation with **Icarus Verilog + GTKWave**
* Firmware built using GCC toolchain for RISC-V

---

## 🔧 Peripherals & IP Cores

### GPIO 🌟

* Memory-mapped at **`0x40000000`**
* Single 32-bit register `GPIO_0` (offset `0x0`)
* Writes update **LED outputs** in `top.v`
* Interface:

| Signal            | Description               |
| ----------------- | ------------------------- |
| `waddr` / `raddr` | 32-bit local addresses    |
| `wdata` / `rdata` | 32-bit data lines         |
| `wen` / `ren`     | Write / Read enable       |
| `wstrb`           | Byte-lane write strobe    |
| `rvalid`          | Read data valid handshake |
| `wready`          | Write accepted            |

**Software access sequence**:

1️⃣ CPU writes 32-bit value to `0x40000000`
2️⃣ `device_select` asserts `s1_sel_gpio` → `gpio_ip`
3️⃣ `gpio_ip` latches bytes according to `wstrb`
4️⃣ Output drives `leds` in `top.v`

---

### UART ✉️

* Memory-mapped at **`0x50000000`**
* Registers:

| Offset | Name    | Type       | Description                                      |
| ------ | ------- | ---------- | ------------------------------------------------ |
| 0x00   | U\_DATA | R/W 32-bit | 8-bit payload to transmit / receive              |
| 0x04   | U\_STAT | R 32-bit   | Status bits: `READY` (bit 5), `TX_DONE` (bit 13) |
| 0x08   | U\_CTRL | W 32-bit   | Control: `START` (bit 9)                         |

**Transmit sequence**:

1️⃣ Poll `U_STAT.READY` (bit 5)
2️⃣ Write byte into `U_DATA` (0x50000000)
3️⃣ Toggle `U_CTRL.START` (0x50000008)
4️⃣ Check `U_STAT.TX_DONE` to confirm byte sent

---

## 🗺️ Address Map

| Address      | Peripheral | Notes                         |
| ------------ | ---------- | ----------------------------- |
| `0x00000000` | Memory     | Firmware `.text` linked here  |
| `0x40000000` | GPIO       | `GPIO_0` 32-bit data register |
| `0x50000000` | UART       | `U_DATA`, `U_STAT`, `U_CTRL`  |

**Device select logic (`device_select.v`)**:

```verilog
wire mem_space  = (addr[31:28] == 4'h0);
wire gpio_space = (addr[31:28] == 4'h4);
wire uart_space = (addr[31:28] == 4'h5);

assign s0_sel_mem  = mem_space;
assign s1_sel_gpio = gpio_space;
assign s2_sel_uart = uart_space;
```

**Multiplexing reads in `top.v`**:

```verilog
case ({s2_sel_uart, s1_sel_gpio, s0_sel_mem})
  3'b001: processor_rdata = mem_rdata;
  3'b010: processor_rdata = rdata_gpio;
  3'b100: processor_rdata = rdata_uart;
endcase
```

---

## 🧩 Firmware & Linker

* Small C program (`main.c`) compiled to ELF → hex for simulation ROM
* Linker script `sections.lds` maps `.text` to **ROM origin `0x00000000`**
* Corsair-generated headers (`gpio_regs.h`, `uart_regs.h`) provide **correct base addresses** and offsets

---

## 🏗️ Top-Level Design (`top.v`)

**ASCII block diagram**:

```
CPU (FemtoRV32)
  |-- instr fetch -> Memory @ 0x00000000
  |-- load/store addr -> device_select
          |-- 0x0... -> Memory
          |-- 0x4... -> GPIO IP
          |-- 0x5... -> UART IP
```



## 🖥️ Simulation & Dependencies

**Required tools**:

* Icarus Verilog (`iverilog`)
* GTKWave (`gtkwave`)
* RISC-V GCC toolchain (`riscv32-unknown-elf-gcc`)
* Python 3 (`makehex.py` script)

**Ubuntu/Debian install**:

```bash
sudo apt update
sudo apt install iverilog gtkwave build-essential python3 python3-pip make
sudo apt install gcc-riscv64-unknown-elf
```

**Run simulation**:

```bash
# Compile RTL + testbench
iverilog -g2012 src/*.v tb_processor.v -o prog

# Run
vvp prog

# Open waveform
gtkwave wave.vcd
```

**Waveform debugging tips**:

* Watch: `mem_addr`, `mem_wdata`, `mem_wmask`, `processor_rdata`
* Verify: `s0_sel_mem`, `s1_sel_gpio`, `s2_sel_uart`
* UART: monitor `U_DATA`, `U_CTRL`, `U_STAT`, `o_uart_tx`

---

## 📝 Usage Guide

1️⃣ Build firmware:

```bash
cd firmware
make
```

2️⃣ Generate hex:

```bash
python3 makehex.py firmware.elf > firmware.hex
```

3️⃣ Run simulation:

```bash
cd ../
iverilog -g2012 src/*.v tb_processor.v -o prog
vvp prog
gtkwave wave.vcd
```

---

## 🌟 Future Development

* I2C & SPI peripheral IPs via Corsair modules
* FPGA hardware validation
* ASIC flow: OpenLane / Qflow → GDS for tapeout

---

## 🙏 Acknowledgments

* FemtoRV32 / PicoRV32 cores for inspiration
* Corsair project for register/IP generation

---

