RISCV_TOOLCHAIN_DIR = /opt/riscv32i/bin
GCC = $(RISCV_TOOLCHAIN_DIR)/riscv32-unknown-elf-gcc
OBJ_DUMP = $(RISCV_TOOLCHAIN_DIR)/riscv32-unknown-elf-objdump
OBJ_COPY = $(RISCV_TOOLCHAIN_DIR)/riscv32-unknown-elf-objcopy
C_SRC_DIR =	.
ASM_SRC_DIR = .
LDS_SRC_DIR = . 
RESULT_DIR = .
MEM_SIZE = 1024

all: $(RESULT_DIR)/firmware.hex $(RESULT_DIR)/dumpfile

$(RESULT_DIR)/firmware.hex: $(RESULT_DIR)/firmware.bin makehex.py
	python3 makehex.py $< $(MEM_SIZE) > $@

$(RESULT_DIR)/firmware.bin: $(C_SRC_DIR)/firmware.elf
	$(OBJ_COPY) -O binary $< $@

$(RESULT_DIR)/dumpfile: $(C_SRC_DIR)/firmware.elf
	$(OBJ_DUMP) -d $^ > $@

$(C_SRC_DIR)/firmware.elf: $(C_SRC_DIR)/main.o $(ASM_SRC_DIR)/start.o sections.lds
	$(GCC) -Og -mabi=ilp32 -march=rv32i -ffreestanding -nostdlib -o $@ -Wl,--build-id=none,-Bstatic,-T,sections.lds,-Map,$(RESULT_DIR)/firmware.map,--strip-debug $(ASM_SRC_DIR)/start.o $(C_SRC_DIR)/main.o -lgcc

$(C_SRC_DIR)/main.o: $(ASM_SRC_DIR)/main.c
	$(GCC) -c -Iinc/ -mabi=ilp32 -march=rv32i -Og --std=c99 -ffreestanding -nostdlib -o $@ $<

$(ASM_SRC_DIR)/start.o: $(ASM_SRC_DIR)/start.S
	$(GCC) -c -mabi=ilp32 -march=rv32i -o $@ $<

clean:
	rm *.o dumpfile *.elf *.hex *.bin *.map
