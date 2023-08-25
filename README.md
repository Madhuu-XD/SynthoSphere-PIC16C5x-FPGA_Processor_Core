# SynthoSphere_PIC16C5x_FPGA_Processor_Core
PIC16C5x-compatible FPGA Processor core

# General Description
This project is provides a synthesizable core implementation of the PIC16C5x microcontroller. As a core, it has a full complement of PIC16C5x RAM and peripherals. However, the user is responsible for providing memory for the core's program and for configuring the core's I/O ports.
 
## Features
The PIC16C5x supports to full instruction set of the PIC16C5x family of microcomputers, and it also incorporates the I/O and timers found in the family. The core also implements all of the RAM normally found in this family of microcomputers.

The timeout period of the watchdog timer is configurable. Although it is normally set to be equal to a 20-bit timer, it can be set to be less than that for simulation purposes. See the test bench for an example of how to change the period of the PIC16C5x core's watch dog timer.

### Implementation
The PIC16C5x core is implemented as a single Verilog source file.
PIC16C5x.v                  -- RTL source file for the PIC16C5x core
tb_PIC16C5x.v               -- Rudimentary testbench

#### Synthesis using YOSYS
The objective is for the PIC16C5x to synthesize into a small FPGA, and to be able to support an operating frequency greater than that commonly supported by a commercial single-chip implementation. The supplied core satisfies both objectives.

Use the respective directives in verilog `$dumpfile` and `$dumpvars` for dumping all the generated values in a given file. Once the files are created we are ready to go for simulation. Now open the terminal in the respective directory where both the design and testbench verilog files are present.
Use the command `iverilog PIC16C5x.v tb_PIC16C5x.v` to compile and simulate both the design and testbench files. Once this is done an "a.out" file is generated.
Now, we have to run the generated a.out file using `./a.out` command. Once this is done, a vcd (value change dump) file which will be used to generate waveforms is created. To see the waveform run 
`gtkwave own_PIC16C5x.vcd` command in the terminal. 

##### Pre-synthesis Waveform 
![waveform using gtkwave command](https://github.com/Madhuu-XD/SynthoSphere-PIC16C5x-FPGA_Processor_Core/blob/main/output/pre_synthesis.png)
The waveform suggests the output values being displayed based on testbench. 

##### Now its time to perform Synthesis!!!
For performing synthesis we have to open yosys, to do so use the command `yosys`. Now we are inside yosys shell and ready to perform synthesis for our design. Before this we have to load the library and this can be done by using the command `read_liberty -lib ../lib/sky130_fd_sc_hd__tt_025C_1v80` . Then, the verilog file must be read by the tool in order to perform synthesis, use `read_verilog PIC16C5x.v` to do so.
Once this is done we are good to go for synthesis!!! Use `synth -top PIC16C5x` to perform synthesis. Now the tool will perform synthesis and generates a netlist which contains the information about the gates and flip-flops used in the design.

###### Post-synthesis 
![](https://github.com/Madhuu-XD/SynthoSphere-PIC16C5x-FPGA_Processor_Core/blob/main/output/post_synthesis.svg)

After this the next step is to write the generated netlist to a file. To do so,
##### Use `write_verilog -noattr netlist_Pic16c5x.v` - This provides a netlist with the comments and redundancies!!
Since the netlist files are generated, exit from yosys. Use `exit` to do so.

#### Synthesis Done!!!

In conclusion, the PIC16C5x supports to full instruction set of the PIC16C5x family of microcomputers, and it also incorporates the I/O and timers found in the family. The core also implements all of the RAM normally found in this family of microcomputers.








    
