# RISC-16-FPGA
This project implements a 16-bit Reduced Instruction Set Computer (RISC) using Verilog HDL and is deployed on the Nexys 4 DDR FPGA board. It simulates a basic processor capable of executing a fixed instruction set with a four-stage execution cycle: Fetch, Decode, Execute, and Update PC.

# Features!

- 16-bit wide instruction and data paths
- Register file with 16 general-purpose registers
- ALU with operations: ADD, SUB, AND, OR, XOR, NOT, SRA, SLA
- Instruction memory and data memory
- Hard-coded instruction ROM
- Memory-mapped output to 7-segment display
- Fully synchronous FSM-based controller

# Instruction Set Implemented

| Mnemonic | Format              | Binary                |
|----------|---------------------|-----------------------|
| LW       | LW rd, imm          | 1001_rd_1100_imm      |
| SW       | SW rs, imm          | 1010_rs_1100_imm      |
| ADD      | ADD rd, rs, rt      | 0000_rd_rs_rt         |
| SUB      | SUB rd, rs, rt      | 0001_rd_rs_rt         |
| LI       | LI rd, imm          | 1000_rd_imm           |
| XOR      | XOR rd, rs, rt      | 0100_rd_rs_rt         |
| SRA      | SRA rd, rs          | 0110_00_rs_rd         |

# Execution Stages

1. **Fetch:** Load instruction from instruction memory using the Program Counter (PC).
2. **Decode:** Parse opcode, source and destination registers, and immediates.
3. **Execute:** ALU performs computation or address calculation.
4. **Writeback / Update PC:** Write result and increment PC.

# Demo Output

Final values stored in memory locations `203`, `204`, and `205` are displayed in decimal on the 7-segment display after execution of the following instruction sequence:
LW 5 201
LW 6 202
ADD 7 5 6
SW 7 203
LI 8 250
SUB 4 8 5
SW 4 204
SRA 3 7
XOR 2 3 4

# Files

- `RISC16_Project.v/` – All Verilog HDL modules
- `RISC16_FPGA_Report_JavierUribe.pdf/` – Full lab report

# Tools Used

- Vivado Design Suite 2024.2
- Digilent Nexys 4 DDR FPGA Board
- Verilog HDL

# Known Issues and Fixes

- **Issue:** Segment flickering during display  
  **Fix:** Updated multiplexing and BCD conversion logic.

- **Issue:** Memory not updating during SW  
  **Fix:** Verified ALU outputs and MEM write logic.

# Author

Javier Uribe – LinkedIn - (https://www.linkedin.com/in/javier-uribe-203852232/))  
Electrical & Computer Engineering Student – UTSA




