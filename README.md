
# 🖥️ Compact-RV32-SoC

**Tiny RISC-V SoC for learning, experimentation, and FPGA prototyping!**
This SoC is built around a **FemtoRV32 CPU**, with **memory**, **GPIO**, and **UART** peripherals. The design is simulation-ready using **Icarus Verilog** and runs simple firmware compiled via **GCC**.
## 🎯 What is This?

**picoSoC_v3** is your gateway to understanding hardware design! It's a minimal yet fully functional RISC-V SoC that you can simulate, learn from, and expand. Perfect for students, hobbyists, and engineers exploring the world of chip design.

```
┌─────────────────────────────────────────┐
│          🖥️  picoSoC_v3 System          │
├─────────────────────────────────────────┤
│  ⚙️  RV32I CPU Core                     │
│  💾 Memory Module (ROM/RAM)             │
│  📡 UART Peripheral                     │
│  💡 GPIO Controller                     │
│  🔌 Memory-Mapped I/O Bus               │
└─────────────────────────────────────────┘
```

---

## ✨ Key Features

| Component | Description |
|-----------|-------------|
| 🧠 **CPU Core** | RV32I-based processor with instruction pipeline |
| 💾 **Memory** | Flash/ROM initialization from hex firmware |
| 📡 **UART** | Serial communication for debugging & data transfer |
| 💡 **GPIO** | LED control & digital I/O operations |
| 🔧 **Bus System** | Clean memory-mapped peripheral access |
| 🧪 **Testbench** | Complete simulation environment included |

---

## 🔌 Peripheral Deep Dive

### 💡 GPIO - Your Digital Output Gateway

```
Memory Write → [Address Decoder] → GPIO Register → LEDs 💡
```

- **Memory-mapped** registers for instant hardware control
- **Write mask** support for selective bit updates
- **Read-back** capability for GPIO state monitoring
- Integrated via `device_select` logic in `top.v`

### 📡 UART - Serial Communication Made Simple

```
CPU ←→ [UART Registers] ←→ TX/RX Lines ←→ 🖥️ Terminal
```

- **Byte-oriented** TX/RX operations
- **Status registers** for ready/busy polling
- **Interrupt support** (configurable)
- Auto-generated register map via Corsair

---

## 🎨 Corsair IP Generation Magic

**Corsair** transforms simple descriptions into production-ready RTL!

```yaml
📝 YAML Description
      ↓
🔮 Corsair Generator
      ↓
📦 Verilog RTL + C Headers
```

### ✅ Benefits:
- 🎯 **Consistency** - RTL and firmware always in sync
- ⚡ **Speed** - Generate peripherals in seconds
- 🛡️ **Reliability** - No manual offset calculations
- 📚 **Documentation** - Auto-generated register maps

### 🔧 How We Used It:
1. Define UART/GPIO registers in YAML
2. Run Corsair generator
3. Get Verilog modules + C header files (`*_regs.h`)
4. Include headers in firmware for instant peripheral access

---

## 🗺️ Memory Architecture

```
┌─────────────────────────────────────┐
│  0x00000000  ┌─────────────────┐   │
│              │  🔒 Boot ROM    │   │
│              │  (Firmware)     │   │
│  0x00001000  ├─────────────────┤   │
│              │  💾 RAM         │   │
│              │  (Data/Stack)   │   │
│  0x10000000  ├─────────────────┤   │
│              │  💡 GPIO        │   │
│  0x10001000  ├─────────────────┤   │
│              │  📡 UART        │   │
│  0x10002000  └─────────────────┘   │
└─────────────────────────────────────┘
```

**Linker Script** (`sections.lds`) ensures:
- ✅ Code lands in ROM region
- ✅ Variables live in RAM
- ✅ Peripherals mapped to correct addresses
- ✅ Interrupt vectors properly placed

---

## 🏗️ System Architecture (`top.v`)

```
        ┌──────────────────────────────────┐
        │          CPU Core 🧠             │
        └────────────┬─────────────────────┘
                     │
            ┌────────┴────────┐
            │  Device Select  │  🚦
            │     Logic       │
            └────────┬────────┘
                     │
        ┌────────────┼────────────┐
        │            │            │
   ┌────▼───┐   ┌───▼────┐  ┌───▼────┐
   │ Memory │   │  GPIO  │  │  UART  │
   │   💾   │   │   💡   │  │   📡   │
   └────────┘   └────────┘  └────────┘
```

**Clean separation of concerns:**
- Memory accesses → Memory block
- I/O accesses → Peripheral blocks  
- `device_select` routes based on address ranges

---

## 🛠️ Required Tools

| Tool | Purpose | Icon |
|------|---------|------|
| **Icarus Verilog** | HDL simulation | 🔬 |
| **GTKWave** | Waveform viewer | 📊 |
| **RISC-V GCC** | Firmware compiler | ⚙️ |
| **Python 3** | Helper scripts | 🐍 |
| **Make** | Build automation | 🏗️ |

### 📦 Quick Install (Ubuntu/Debian)

```bash
sudo apt update
sudo apt install iverilog gtkwave build-essential \
                 python3 python3-pip make
```

*For RISC-V toolchain:* Download from [SiFive](https://www.sifive.com/software) or build from source

---

## 🚀 Quick Start Guide

### 1️⃣ Build Firmware

```bash
cd firmware
make
# Generates: firmware.elf & firmware.hex
```

### 2️⃣ Run Simulation

```bash
cd /home/iraj/LearnSoC/picoSoC_v3
iverilog -g2012 src/*.v tb_processor.v -o prog
vvp prog
```

### 3️⃣ View Waveforms

```bash
gtkwave wave.vcd
```

**🎬 Expected Output:**
```
💡 LED blinks in simulation
📡 UART transmits "Hello World!"
✅ All tests pass
```

---

## 🔮 Future Roadmap

### 🚧 Coming Soon:
- [ ] 🔗 **I2C** peripheral (sensor interfaces)
- [ ] 🔄 **SPI** peripheral (flash memory, SD cards)
- [ ] 🔌 **Interrupt controller** (enhanced responsiveness)

### 🎯 Long-Term Goals:
- [ ] 🎮 **FPGA deployment** (real hardware testing)
- [ ] 🏭 **ASIC flow** integration (OpenLane/Qflow)
- [ ] 📐 **GDS generation** for tapeout-ready design

---

## 🧪 Troubleshooting

| Issue | Solution |
|-------|----------|
| ❌ Elaboration errors | Check for `use-before-declare` in Verilog |
| ❌ Firmware won't build | Verify RISC-V toolchain path |
| ❌ No waveform output | Ensure testbench has `$dumpfile()` |
| ❌ Peripheral not responding | Check address mapping in linker script |

---

## 🙏 Acknowledgments

- **FemtoRV32/PicoRV32** teams for compact core inspiration
- **Corsair** project for register generation tools
- **RISC-V Foundation** for the open ISA
- **Open-source community** for EDA tools

---

## 📚 Learn More

```
📖 Documentation → /docs
🔧 Examples      → /firmware/examples  
🧪 Testbenches   → /testbench
🎨 Scripts       → /scripts
```

---

<div align="center">

### ⭐ Star this repo if you found it helpful!

**Made with ❤️ for the hardware hacking community**

🔗 [Report Bug](issues) • 💡 [Request Feature](issues) • 📧 [Contact](mailto:you@example.com)

</div>
