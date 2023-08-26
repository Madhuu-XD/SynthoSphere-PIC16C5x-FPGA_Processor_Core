# SynthoSphere_PIC16C5x_FPGA_Processor_Core
PIC16C5x-compatible FPGA Processor core

# General Description
This project is provides a synthesizable core implementation of the PIC16C5x microcontroller. As a core, it has a full complement of PIC16C5x RAM and peripherals. However, the user is responsible for providing memory for the core's program and for configuring the core's I/O ports.
 
## Features
The PIC16C5x supports to full instruction set of the PIC16C5x family of microcomputers, and it also incorporates the I/O and timers found in the family. The core also implements all of the RAM normally found in this family of microcomputers.

The timeout period of the watchdog timer is configurable. Although it is normally set to be equal to a 20-bit timer, it can be set to be less than that for simulation purposes. See the test bench for an example of how to change the period of the PIC16C5x core's watch dog timer.

### Implementation
The PIC16C5x core is implemented as a single Verilog source file.
1. PIC16C5x.v                  -- RTL source file for the PIC16C5x core
1. tb_PIC16C5x.v               -- Rudimentary testbench

#### Testbench
```
`timescale 1ns / 1ps

module tb_PIC16C5x_v;

	// UUT Module Ports
	reg     POR;

	reg     Clk;
	reg     ClkEn;

	wire    [11:0] PC;
	reg     [11:0] IR;

	wire    [3:0] TRISA;
	wire    [3:0] PORTA;
	reg     [3:0] PA_DI;
 
        wire    [7:0] TRISB;
	wire    [7:0] PORTB;
	reg     [7:0] PB_DI;

	wire    [7:0] TRISC;
	wire    [7:0] PORTC;
	reg     [7:0] PC_DI;

	reg     MCLR;
	reg     T0CKI;
	reg     WDTE;

	// UUT Module Test Ports
    
	wire    Err;
	
        wire    [5:0] OPTION;

        wire    [ 8:0] dIR;
	wire    [11:0] ALU_Op;
	wire    [ 8:0] KI;
	wire    Skip;

 	wire    [11:0] TOS;
	wire    [11:0] NOS;

	wire    [7:0] W;

	wire    [6:0] FA;
	wire    [7:0] DO;
	wire    [7:0] DI;

	wire    [7:0] TMR0;
	wire    [7:0] FSR;
	wire    [7:0] STATUS;
	
        wire    T0CKI_Pls;
	
        wire    WDTClr;
	wire    [9:0] WDT;
	wire    WDT_TC;
	wire    WDT_TO;
	
        wire    [7:0] PSCntr;
	wire    PSC_Pls;
    
	// Instantiate the Unit Under Test (UUT)
	PIC16C5x #(.WDT_Size(10))
             uut (.POR(POR), 
                  .Clk(Clk), 
                  .ClkEn(ClkEn), 
                  .PC(PC), 
                  .IR(IR), 
                  .TRISA(TRISA), 
                  .PORTA(PORTA), 
                  .PA_DI(PA_DI), 
                  .TRISB(TRISB), 
                  .PORTB(PORTB), 
                  .PB_DI(PB_DI), 
                  .TRISC(TRISC), 
                  .PORTC(PORTC), 
                  .PC_DI(PC_DI), 
                  .MCLR(MCLR), 
                  .T0CKI(T0CKI), 
                  .WDTE(WDTE), 
                  .Err(Err), 
                  .OPTION(OPTION), 
                  .dIR(dIR), 
                  .ALU_Op(ALU_Op), 
                  .KI(KI), 
                  .Skip(Skip), 
                  .TOS(TOS), 
                  .NOS(NOS), 
                  .W(W), 
                  .FA(FA), 
                  .DO(DO), 
                  .DI(DI), 
                  .TMR0(TMR0), 
                  .FSR(FSR), 
                  .STATUS(STATUS), 
                  .T0CKI_Pls(T0CKI_Pls), 
                  .WDTClr(WDTClr), 
                  .WDT(WDT), 
                  .WDT_TC(WDT_TC), 
                  .WDT_TO(WDT_TO), 
                  .PSCntr(PSCntr), 
                  .PSC_Pls(PSC_Pls));

	initial begin
		// Initialize Inputs
	        $dumpfile("own_PIC16C5x.vcd");   
	        $dumpvars(0,tb_PIC16C5x_v);		
		POR     = 1;
		Clk     = 1;
        ClkEn   = 1;
		IR      = 0;
		PA_DI   = 0;
		PB_DI   = 0;
		PC_DI   = 0;
		MCLR    = 0;
		T0CKI   = 0;
		WDTE    = 1;

		// Wait 100 ns for global reset to finish
		#101;

        POR = 0;
//        ClkEn = 1;

        #50 $finish;        
	end
    
    always #5 Clk = ~Clk;
    
    
    // Test Program ROM
      
    always @(PC or POR)
    begin
        if(POR)
            IR <= 12'b0000_0000_0000;    // NOP             ;; No Operation
        else 
            case(PC[11:0])
                12'h000 : IR = 12'b0111_0110_0011;   // BTFSS   0x03,3  ;; Test PD (STATUS.3), if set, not SLEEP restart
                12'h001 : IR = 12'b1010_0011_0000;   // GOTO    0x030   ;; SLEEP restart, continue test program
                12'h002 : IR = 12'b1100_0000_0111;   // MOVLW   0x07    ;; load OPTION
                12'h003 : IR = 12'b0000_0000_0010;   // OPTION
                12'h004 : IR = 12'b0000_0100_0000;   // CLRW            ;; clear working register
                12'h005 : IR = 12'b0000_0000_0101;   // TRISA           ;; load W into port control registers
                12'h006 : IR = 12'b0000_0000_0110;   // TRISB
                12'h007 : IR = 12'b0000_0000_0111;   // TRISC
                12'h008 : IR = 12'b1010_0000_1010;   // GOTO    0x00A   ;; Test GOTO
                12'h009 : IR = 12'b1100_1111_1111;   // MOVLW   0xFF    ;; instruction should be skipped
                12'h00A : IR = 12'b1001_0000_1101;   // CALL    0x0D    ;; Test CALL
                12'h00B : IR = 12'b0000_0010_0010;   // MOVWF   0x02    ;; Test Computed GOTO, Load PCL with W
                12'h00C : IR = 12'b0000_0000_0000;   // NOP             ;; No Operation
                12'h00D : IR = 12'b1000_0000_1110;   // RETLW   0x0E    ;; Test RETLW, return 0x0E in W
                12'h00E : IR = 12'b1100_0000_1001;   // MOVLW   0x09    ;; starting RAM + 1
                12'h00F : IR = 12'b0000_0010_0100;   // MOVWF   0x04    ;; indirect address register (FSR)

                12'h010 : IR = 12'b1100_0001_0111;   // MOVLW   0x17    ;; internal RAM count - 1
                12'h011 : IR = 12'b0000_0010_1000;   // MOVWF   0x08    ;; loop counter
                12'h012 : IR = 12'b0000_0100_0000;   // CLRW            ;; zero working register
                12'h013 : IR = 12'b0000_0010_0000;   // MOVWF   0x00    ;; clear RAM indirectly
                12'h014 : IR = 12'b0010_1010_0100;   // INCF    0x04,1  ;; increment FSR
                12'h015 : IR = 12'b0010_1110_1000;   // DECFSZ  0x08,1  ;; decrement loop counter
                12'h016 : IR = 12'b1010_0001_0011;   // GOTO    0x013   ;; loop until loop counter == 0
                12'h017 : IR = 12'b1100_0000_1001;   // MOVLW   0x09    ;; starting RAM + 1
                12'h018 : IR = 12'b0000_0010_0100;   // MOVWF   0x04    ;; reload FSR
                12'h019 : IR = 12'b1100_1110_1001;   // MOVLW   0xE9    ;; set loop counter to 256 - 23
                12'h01A : IR = 12'b0000_0010_1000;   // MOVWF   0x08
                12'h01B : IR = 12'b0010_0000_0000;   // MOVF    0x00,0  ;; read memory into W 
                12'h01C : IR = 12'b0011_1110_1000;   // INCFSZ  0x08,1  ;; increment counter loop until 0
                12'h01D : IR = 12'b1010_0001_1011;   // GOTO    0x01B   ;; loop    
                12'h01E : IR = 12'b0000_0000_0100;   // CLRWDT          ;; clear WDT
                12'h01F : IR = 12'b0000_0110_1000;   // CLRF    0x08    ;; Clear Memory Location 0x08

                12'h020 : IR = 12'b0010_0110_1000;   // DECF    0x08,1  ;; Decrement Memory Location 0x08
                12'h021 : IR = 12'b0001_1100_1000;   // ADDWF   0x08,0  ;; Add Memory Location 0x08 to W, Store in W
                12'h022 : IR = 12'b0000_1010_1000;   // SUBWF   0x08,1  ;; Subtract Memory Location 0x08
                12'h023 : IR = 12'b0011_0110_1000;   // RLF     0x08,1  ;; Rotate Memory Location 0x08
                12'h024 : IR = 12'b0011_0010_1000;   // RRF     0x08,1  ;; Rotate Memory Location
                12'h025 : IR = 12'b1100_0110_1001;   // MOVLW   0x69    ;; Load W with test pattern: W <= 0x69
                12'h026 : IR = 12'b0000_0010_1000;   // MOVWF   0x08    ;; Initialize Memory with test pattern
                12'h027 : IR = 12'b0011_1010_1000;   // SWAPF   0x08,1  ;; Test SWAPF: (0x08) <= 0x96 
                12'h028 : IR = 12'b0001_0010_1000;   // IORWF   0x08,1  ;; Test IORWF: (0x08) <= 0x69 | 0x96 
                12'h029 : IR = 12'b0001_0110_1000;   // ANDWF   0x08,1  ;; Test ANDWF: (0x08) <= 0x69 & 0xFF
                12'h02A : IR = 12'b0001_1010_1000;   // XORWF   0x08,1  :: Test XORWF: (0x08) <= 0x69 ^ 0x69
                12'h02B : IR = 12'b0010_0110_1000;   // COMF    0x08    ;; Test COMF:  (0x08) <= ~0x00  
                12'h02C : IR = 12'b1101_1001_0110;   // IORLW   0x96    ;; Test IORLW:      W <= 0x69 | 0x96
                12'h02D : IR = 12'b1110_0110_1001;   // ANDLW   0x69    ;; Test ANDLW:      W <= 0xFF & 0x69
                12'h02E : IR = 12'b1111_0110_1001;   // XORLW   0x69    ;; Test XORLW:      W <= 0x69 ^ 0x69
                12'h02F : IR = 12'b0000_0000_0011;   // SLEEP           ;; Stop Execution of test program: HALT

                12'h030 : IR = 12'b0000_0000_0100;   // CLRWDT          ;; Detected SLEEP restart, Clr WDT to reset PD
                12'h031 : IR = 12'b0110_0110_0011;   // BTFSC   0x03,3  ;; Check STATUS.3, skip if ~PD clear
                12'h032 : IR = 12'b1010_0011_0100;   // GOTO    0x034   ;; ~PD is set, CLRWDT cleared PD
                12'h033 : IR = 12'b1010_0011_0011;   // GOTO    0x033   ;; ERROR: hold here on error
                12'h034 : IR = 12'b1100_0001_0000;   // MOVLW   0x10    ;; Load FSR with non-banked RAM address
                12'h035 : IR = 12'b0000_0010_0100;   // MOVWF   0x04    ;; Initialize FSR for Bit Processor Tests
                12'h036 : IR = 12'b0000_0110_0000;   // CLRF    0x00    ;; Clear non-banked RAM location using INDF
                12'h037 : IR = 12'b0101_0000_0011;   // BSF     0x03,0  ;; Set   STATUS.0 (C) bit 
                12'h038 : IR = 12'b0100_0010_0011;   // BCF     0x03,1  ;; Clear STATUS.1 (DC) bit
                12'h039 : IR = 12'b0100_0100_0011;   // BCF     0x03,2  ;; Clear STATUS.2 (Z) bit
                12'h03A : IR = 12'b0010_0000_0011;   // MOVF    0x03,0  ;; Load W with STATUS
                12'h03B : IR = 12'b0011_0000_0000;   // RRF     0x00,0  ;; Rotate Right RAM location: C <= 0,      W <= 0x80
                12'h03C : IR = 12'b0011_0110_0000;   // RLF     0x00,0  ;; Rotate Left  RAM location: C <= 0, (INDF) <= 0x00
                12'h03D : IR = 12'b0000_0010_0000;   // MOVWF   0x00    ;; Write result back to RAM: (INDF) <= 0x80
                12'h03E : IR = 12'b0000_0010_0001;   // MOVWF   0x01    ;; Write to TMR0, clear Prescaler
                12'h03F : IR = 12'b1010_0000_0000;   // GOTO    0x000   ;; Restart Program

                default : IR = 12'b0000_0000_0000;   // NOP             ;; Reset Vector: Jump 0x000 (Start)
            endcase
    end
    
endmodule
```

#### Compilation
The objective is for the PIC16C5x to synthesize into a small FPGA, and to be able to support an operating frequency greater than that commonly supported by a commercial single-chip implementation. The supplied core satisfies both objectives.

Use the respective directives in verilog `$dumpfile` and `$dumpvars` for dumping all the generated values in a given file. Once the files are created we are ready to go for simulation. Now open the terminal in the respective directory where both the design and testbench verilog files are present.
Use the command `iverilog PIC16C5x.v tb_PIC16C5x.v` to compile and simulate both the design and testbench files. Once this is done an "a.out" file is generated.
Now, we have to run the generated a.out file using `./a.out` command. Once this is done, a vcd (value change dump) file which will be used to generate waveforms is created. To see the waveform run 
`gtkwave own_PIC16C5x.vcd` command in the terminal. 

#### Pre-synthesis Waveform 
![waveform using gtkwave command](https://github.com/Madhuu-XD/SynthoSphere-PIC16C5x-FPGA_Processor_Core/blob/main/output/pre_synthesis.png)
The waveform suggests the output values being displayed based on testbench. 

#### Synthesis using YOSYS
![The main yosys window is shown](https://github.com/Madhuu-XD/SynthoSphere-PIC16C5x-FPGA_Processor_Core/blob/main/output/yosys.png)

#### Now its time to perform Synthesis!!!
For performing synthesis we have to open yosys, to do so use the command `yosys`. Now we are inside yosys shell and ready to perform synthesis for our design. Before this we have to load the library and this can be done by using the command `read_liberty -lib ../lib/sky130_fd_sc_hd__tt_025C_1v80` . Then, the verilog file must be read by the tool in order to perform synthesis, use `read_verilog PIC16C5x.v` to do so.
Once this is done we are good to go for synthesis!!! Use `synth -top PIC16C5x` to perform synthesis. Now the tool will perform synthesis and generates a netlist which contains the information about the gates and flip-flops used in the design.

#### Yosys commands
```
$yosys
yosys> read_liberty -lib <relative or abs path>/ lib file 
yosys> read_verilog <verilog_file.v>
yosys> synth -top <verilog_file> 
yosys> abc -liberty <relative or abs path>/ lib file ( generates results on ur design → netlist verify them before continuing)
yosys> show 
yosys> write_verilog <file_name>.v  OR    write_verilog -noattr  <file_name>.v
```

#### Post Synthesis Yosys commands
![Post Synthesis intermediate result 1](https://github.com/Madhuu-XD/SynthoSphere-PIC16C5x-FPGA_Processor_Core/blob/main/output/yosys-0.png)

#### Post Synthesis intermediate result 1
![Post Synthesis intermediate result 1](https://github.com/Madhuu-XD/SynthoSphere-PIC16C5x-FPGA_Processor_Core/blob/main/output/yosys-1.png)

#### Post Synthesis intermediate result 2
![Post Synthesis intermediate result 2](https://github.com/Madhuu-XD/SynthoSphere-PIC16C5x-FPGA_Processor_Core/blob/main/output/yosys-2.png)

##### Post-synthesis 
![](https://github.com/Madhuu-XD/SynthoSphere-PIC16C5x-FPGA_Processor_Core/blob/main/output/post_synthesis.png)

After this the next step is to write the generated netlist to a file. To do so,

#### Netlist generation
```
write_verilog -noattr netlist_Pic16c5x.v
```
This provides a netlist with the comments and redundancies.
Since the netlist files are generated, exit from yosys. Use `exit` to do so.

#### Synthesis Done!!!
In conclusion, the PIC16C5x supports to full instruction set of the PIC16C5x family of microcomputers, and it also incorporates the I/O and timers found in the family. The core also implements all of the RAM normally found in this family of microcomputers.
