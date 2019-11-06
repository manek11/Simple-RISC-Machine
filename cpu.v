module cpu(clk,reset, read_data, mem_cmd, mem_addr, write_data, isHalt);

    input clk, reset;
    input [15:0] read_data;
    output wire [1:0] mem_cmd;
    output wire [8:0] mem_addr;
    output wire [15:0] write_data;

    //TODO: isHalt ADDED for lab8
    output wire isHalt;


//Load enabled circuit Instruction register
    wire [15:0] in, iregout;
    assign in = read_data;
    wire load_ir;
    LEC #(16) ireg(clk,load_ir,in,iregout);

    /*
    wire [8:0] PC;
    assign PC = mem_addr;
    */

//Instantiating Instruction Decoder
    wire [2:0] nsel;
    wire [2:0] opcode;
    wire [1:0] op;
    wire [1:0] ALUop, shift;
    wire [15:0] sximm5, sximm8;
    wire [2:0] readnum, writenum;
    wire [2:0] cond;

    idecoder D1(iregout, nsel, opcode, op, ALUop, sximm5, sximm8, shift, readnum, writenum, cond);

    //FSM
    wire loada,loadb,loadc,loads, asel,bsel,PCsel, write;
    wire [1:0] vsel;
    wire load_pc, reset_pc, load_addr, addr_sel;
    FSM FSM(clk,reset,opcode,op, loada,loadb,loadc,loads, asel,bsel,vsel,nsel,write,  load_pc, reset_pc, load_addr, load_ir, addr_sel, mem_cmd, isHalt, PCsel);

//Datapath
//Note you will have to modify datapath to connect proper wires
    wire [15:0] mdata, datapath_out;
    wire [2:0] Zstatus;
    assign mdata = read_data;

    wire [8:0] PC;

    /* wire [8:0] pcOut;
     assign PC = pcOut;
 */

    reg [8:0] BXPC;
    wire [8:0] DPPC;
    assign DPPC = BXPC;
    datapath DP(clk, mdata, sximm8, DPPC, sximm5, datapath_out, Zstatus, vsel, asel, bsel,
        ALUop, loada, loadb, loadc, loads,
        writenum, readnum, write, shift); //Instantiating Datapath


    wire V = Zstatus [2]; // Overflow
    wire N = Zstatus [1]; //negative
    wire Z = Zstatus [0]; //Status

    //Instantiating PC and DataAddress
    //If wire is made called next pc which is the output(next_pc) of the FSM then it makes sense

    wire [8:0] next_pc;
    reg [8:0] PCloop;

    ///*
    always @(*) begin
        casex({opcode, op, cond})

            //B<Label>
            {3'b001, 2'b00, 3'b000}:
                begin
                    PCloop = PC + 1 + sximm8[8:0];
                end

            //BEQ<Label>
            {3'b001, 2'b00, 3'b001}:
                begin
                    if(Z==1'b1) begin
                        PCloop = PC + 1 + sximm8[8:0];
                    end

                    else begin
                        PCloop = PC + 1;
                    end
                end

            //BNE<Label>
            {3'b001, 2'b00, 3'b001}:
                begin
                    if(Z==1'b0) begin
                        PCloop = PC + 1 + sximm8[8:0];
                    end
                    else begin
                        PCloop = PC + 1;
                    end
                end

            //BLT<Label>
            {3'b001, 2'b00, 3'b011}:
                begin
                    if(N!==V) begin
                        PCloop = PC + 1 + sximm8[8:0];
                    end
                    else begin
                        PCloop = PC + 1;
                    end
                end

            //BLE <Label>
            {3'b001, 2'b00, 3'b100}:
                begin
                    if(N!==V | Z==1'b1) begin
                        PCloop = PC + 1 + sximm8[8:0];
                    end
                    else begin
                        PCloop = PC + 1;
                    end
                end

            //BL <Label>
            {3'b010, 2'b11, 3'bxxx}:
                begin
                        PCloop <= PC + 1 + sximm8[8:0];
                end



            default: PCloop = PC+1;

        endcase
    end
    //*/

    //TODO: PC MODIFICATION
    //pcOut changed to PC
    LEC #(9) PC1(clk,load_pc,next_pc,PC);
    wire [8:0] temp_PC = (PCsel ? datapath_out[8:0]+1 : PCloop);
    assign next_pc = reset_pc ? 9'b0 : temp_PC;


    always @(opcode, op) begin

        case({opcode, op})
            5'b01011: BXPC = PC;
            default: BXPC = 9'bz;
        endcase
    end





    //DATA ADDRESS
    wire [8:0] dataAddressOut;
    wire [8:0] dataAddressIn;

    assign dataAddressIn = datapath_out[8:0];

    LEC #(9) DA(clk,load_addr,dataAddressIn,dataAddressOut);
    assign mem_addr = addr_sel ? PC : dataAddressOut;


    assign write_data = datapath_out;

    //Z[0]: representing status as in lab5
    //Z[1]: representing "negative flag" MSB of out = 1, then set to 1, otherwise 0
    //Z[2]: represent overflow tag


endmodule //cpu

//-------------------------HELPER MODULES-------------------------//

//Instruction Decoder
module idecoder(iregout, nsel,  opcode, op, ALUop, sximm5, sximm8, shift, readnum, writenum, cond);
    input [15:0] iregout;
    input [2:0] nsel;
    output [2:0] opcode;
    output [1:0] op;
    output [1:0] ALUop,  shift;
    output [15:0] sximm5, sximm8;
    output [2:0] readnum, writenum;

    //new output cond for conditional branch
    output [2:0]cond;

    //To controller FSM
    wire [2:0] opcode;
    wire [1:0] op;

    //new
    wire [2:0] cond;


    assign opcode = iregout[15:13];
    assign op = iregout[12:11];

    //new
    assign cond = iregout[10:8];



    //From controller FSM
    reg [2:0] Rm;
    reg [2:0] Rd;
    reg [2:0] Rn;

    always @(*) begin
        Rm = iregout[2:0];
        Rd = iregout[7:5];
        Rn = iregout[10:8];
    end

    reg [2:0] readnum;
    reg[2:0] writenum;


    always @(*) begin
        case (nsel)
            3'b001: {readnum, writenum} = {2{Rn}};
            3'b010: {readnum, writenum} = {2{Rm}};
            default: {readnum, writenum} = {2{Rd}};
        endcase
    end

    wire [1:0] shift;
    wire [7:0] imm8;
    wire [4:0] imm5;
    wire [1:0] ALUop;

    //Modifying the shift: when STR, shift will be 2'b00;
    assign shift = (opcode !== 3'b100 || (opcode !== 3'b010 && op == 2'b00)) ? iregout [4:3] : 2'b00;

    //First 8 bits of the instruction register
    assign imm8 = iregout [7:0];
    //assign imm5 = iregout [4:0]; //modification for STR
    assign ALUop = iregout [12:11];

    assign imm5 = iregout [4:0];

    reg [15:0] sximm8;
    reg [15:0] sximm5;

    //Lab-8
    always @(*) begin
        if(imm8[7] == 1'b1) begin
            sximm8 = {8'b11111111,imm8};
        end else begin
            sximm8 = {8'b00000000,imm8};
        end
        if(imm5[4] == 1'b1) begin
            sximm5 = {11'b11111111111,imm5};
        end else begin
            sximm5 = {11'b00000000000,imm5};
        end
    end
endmodule//idecoder

/*
SPECIFICATION:
    nsel:
        case (nsel)
        3'b001: {readnum, writenum} = {2{Rn}};
        3'b010: {readnum, writenum} = {2{Rm}};
        default: {readnum, writenum} = {2{Rd}};

    vsel:
        vsel = 2'b11 -> mdata
        vsel = 2'b10 -> sximm8
        vsel = 2'b01 -> {8'b0,PC}
        vsel = 2'b00 -> C

    mem_cmd:
        `define MREAD   2'b10
        `define MWRITE  2'b01
*/

module FSM(clk,reset,opcode,op, loada,loadb,loadc,loads, asel,bsel,vsel,nsel, write,  load_pc, reset_pc, load_addr, load_ir, addr_sel, mem_cmd, isHalt, PCsel);

//s,w is not needed anymore
    input clk,reset;
    input[2:0] opcode;
    input[1:0] op;

    //ADDED for LAB8
    output reg isHalt;
    output reg PCsel;

//new outputs->
    output reg load_pc;
    output reg reset_pc;
    output reg load_addr;
    output reg load_ir;
    output reg addr_sel;
    output reg [1:0] mem_cmd;

//w, loada,loadb,loadc,loads, asel,bsel,vsel,nsel, mdata, PC, write
//11 + 16 + 8

    output[2:0] nsel;
//output w; Not required because we dont have wait state anymore
    output asel, bsel, loada, loadb, loadc;
    output [1:0] vsel;
    output loads, write;


    reg w;//tell autograder if the statemachine is or is not in the wait state
    reg[2:0] nsel; // 3 bits because selection of 3 different input in MUX
    reg asel;
    reg bsel;
    reg [1:0] vsel;
    reg loada;
    reg loadb;
    reg loadc;
    reg loads;
    reg write;
    reg [26:0] caseOut; // for casex, 26 bits in total, 6 bits for states, and additional 1 bit for PCsel


    `define S 6
    `define SRESET       6'b000000
    `define SIF1         6'b000001
    `define SIF2         6'b000010
    `define SUpdatePC    6'b000011

    `define SDEC         6'b000100

//ALU operations
    `define SALU     6'b000101
    `define SA_LOADA 6'b000110
    `define SA_LOADB 6'b000111
    `define SA_OP    6'b001000

//MOV
    `define SMOVE    6'b001001
    `define SM_LOADB 6'b001010
    `define SM_OP    6'b001011
//`define WRITE--------

//WRITE
    `define WRITE8   6'b001100
    `define WRITE    6'b001101

//New changes for Stage 2
    `define LDR     6'b001110
    `define LDR1    6'b001111
    //              6'b010000
    `define B1      6'b010000
    `define CALL    6'b010001
    //              6'b010001

    `define HALT    6'b010010

    `define LDR2    6'b010011
    `define LDR3    6'b010100
    `define LDR4    6'b010101
    `define WRITEC  6'b010000

    `define STR     6'b010110
    `define STR1     6'b010111
    `define STR2     6'b011000
    `define STR3     6'b011001
    `define STR4     6'b011010
    `define STR5     6'b011011



    //`define XXX         6'b011100
    `define BX      6'b011101
    `define BL     6'b011110
    //`define PCWRITE1    6'b011111

    `define BX1     6'b100000
    `define BX2     6'b100001
    `define BX3     6'b100010
    `define BX4     6'b100011
    //`define PCREAD3     6'b100010
    //`define PCREAD4     6'b100011


    /*
    For HALT instruction:
    Cause program counter to no longer be updated. Implement HALT by adding a state reached from decode when opcode = 111 which loops back to itself
    From this you reset
    */

    wire [5:0] present_state, state_next_reset;
    reg [5:0] state_next;

// state DFF for control FSM for exponent circuit
    vDFF #(6) STATE(clk,state_next_reset,present_state);
// reset mux for control FSM for exponent circuit
    assign state_next_reset = reset ? `SRESET  : state_next;


    //Modified FSM for lab7, accounts for reset,F1,F2,UpdatePC,LDR, and STR instructions are supported
    always @(*) begin
        casex({present_state, opcode, op})
//                          {state_next,loada,loadb,loadc,loads,  asel,bsel,vsel,nsel,write,  load_pc,reset_pc,load_addr,load_ir,addr_sel, mem_cmd,isHalt,PCsel} = caseOut;
            //first 4 traversal states
            {`SRESET,3'bxxx,2'bxx}:
                caseOut = {`SIF1, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0 , 1'b0, 2'b0, 3'b000, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 2'b00, 1'b0,1'b0};

            {`SIF1,3'bxxx,2'bxx}:
                caseOut =  {`SIF2, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0 , 1'b0, 2'b0, 3'b000, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 2'b10, 1'b0,1'b0};


            {`SIF2,3'bxxx,2'bxx}:
                caseOut = {`SUpdatePC, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0 , 1'b0, 2'b00, 3'b000, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b1, 2'b10, 1'b0,1'b0}; //Last 2 bits set to R?

            {`SUpdatePC,3'bxxx,2'bxx}:
                caseOut = {`SDEC, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0 , 1'b0, 2'b00, 3'b000, 1'b0, 1'b1, 1'b0, 1'b1, 1'b0, 1'b0, 2'b00, 1'b0,1'b0};

            //Decode States
            {`SDEC,3'b101,2'bxx}:
                caseOut = {`SALU,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,3'b000,1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 2'b00, 1'b0,1'b0}; // opcode=101, then go SALU

            {`SDEC,3'b110,2'bxx}:
                caseOut = {`SMOVE,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,3'b000,1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 2'b00, 1'b0,1'b0}; // opcode=110, then go SMOVE

            {`SDEC,3'b011,2'bxx}:
                caseOut = {`LDR,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,3'b000,1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 2'b00, 1'b0,1'b0}; // opcode=011, then go to LDR

            {`SDEC,3'b100,2'bxx}:
                caseOut = {`STR,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,3'b000,1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 2'b00, 1'b0,1'b0}; // opcode=100, then go to STR

            //THIS is for B instructions where you need to updatePC
            {`SDEC,3'b001,2'bxx}:
                caseOut = {`B1,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,3'b000,1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 2'b00, 1'b0,1'b0}; // opcode=001, then go to B1

            {`SDEC,3'b010,2'b11}:
                caseOut = {`BL,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,3'b000,1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 2'b00, 1'b0,1'b0}; // opcode=010, then go to Call function
            {`SDEC,3'b010,2'b00}:
                caseOut = {`BX,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,3'b000,1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 2'b00, 1'b0,1'b0}; // opcode=010, then go to Call function
            //{`SDEC,3'b010,2'b10}:
                //caseOut = {`PCWRITE,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,3'b000,1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 2'b00, 1'b0,1'b0}; // opcode=010, then go to Call function

            {`SDEC,3'b111,2'bxx}: caseOut = {`HALT,1'b0,1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,3'b000,1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 2'b00, 1'b0,1'b0}; // opcode=111, then go to HALT

//                          {state_next,loada,loadb,loadc,loads,  asel,bsel,vsel,nsel,write,  load_pc,reset_pc,load_addr,load_ir,addr_sel, mem_cmd,isHalt,PCsel} = caseOut;

            {`SALU,3'b101,2'bxx}: caseOut = {`SA_LOADA,1'b0,1'b0,1'b0,1'b0,    1'b0,1'b0,2'b00,3'b000,1'b0,   1'b0,1'b0,1'b0,1'b0,1'b0,2'b00, 1'b0,1'b0}; // if ps=SALU, then LOADA

            {`SA_LOADA,3'b101,2'bxx}: caseOut = {`SA_LOADB,1'b1,1'b0,1'b0,1'b0,   1'b0,1'b0,2'b00,3'b001,1'b0,   1'b0,1'b0,1'b0,1'b0,1'b0,2'b00, 1'b0,1'b0};

            {`SA_LOADB,3'b101,2'bxx}: caseOut = {`SA_OP,1'b0,1'b1,1'b0,1'b0,   1'b0,1'b0,2'b00,3'b010,1'b0,   1'b0,1'b0,1'b0,1'b0,1'b0,2'b00, 1'b0,1'b0};

            {`SA_OP,3'b101,2'b01}: caseOut = {`SIF1 ,1'b0,1'b0,1'b0,1'b1,   1'b0,1'b0,2'b00,3'b000,1'b0,   1'b0,1'b0,1'b0,1'b0,1'b0,2'b00, 1'b0,1'b0};

            {`SA_OP,3'b101,2'bxx}: caseOut = {`WRITE,1'b0,1'b0,1'b1,1'b1,   1'b0,1'b0,2'b00,3'b000,1'b0,   1'b0,1'b0,1'b0,1'b0,1'b0,2'b00, 1'b0,1'b0};


            //MOVE
            //NODE2: WRITE OR MOVWRITE
            {`SMOVE,3'b110,2'b10}: caseOut = {`WRITE8,1'b0,1'b0,1'b0,1'b0,  1'b0,1'b0,2'b00,3'b000,1'b0,  1'b0,1'b0,1'b0,1'b0,1'b0,2'b00, 1'b0,1'b0};
            {`SMOVE,3'b110,2'b00}: caseOut = {`SM_LOADB,1'b0,1'b0,1'b0,1'b0,  1'b0,1'b0,2'b00,3'b000,1'b0,  1'b0,1'b0,1'b0,1'b0,1'b0,2'b00, 1'b0,1'b0};
            {`SM_LOADB,3'b110,2'bxx}: caseOut = {`SM_OP,1'b0,1'b1,1'b0,1'b0,  1'b0,1'b0,2'b00,3'b010,1'b0,  1'b0,1'b0,1'b0,1'b0,1'b0,2'b00, 1'b0,1'b0};
            {`SM_OP,3'b110,2'bxx}: caseOut = {`WRITE,1'b0,1'b0,1'b1,1'b0,  1'b1,1'b0,2'b00,3'b000,1'b0,  1'b0,1'b0,1'b0,1'b0,1'b0,2'b00, 1'b0,1'b0};

            //                        {state_next,loada,loadb,loadc,loads,asel,bsel,vsel,nsel,write, load_pc, reset_pc, load_addr, load_ir, addr_sel, mem_cmd,isHalt,PCsel} = caseOut;

            //WRITE if1 added reset removed
            {`WRITE8,3'bxxx,2'bxx}: caseOut = {`SIF1,1'b0,1'b0,1'b0,1'b0,  1'b0,1'b0,2'b10,3'b001,1'b1,  1'b0,1'b0,1'b0,1'b0,1'b0,2'b00, 1'b0,1'b0};
            {`WRITE,3'bxxx,2'bxx}: caseOut = {`SIF1 ,1'b0,1'b0,1'b0,1'b0,  1'b0,1'b0,2'b00,3'b100,1'b1,  1'b0,1'b0,1'b0,1'b0,1'b0,2'b00, 1'b0,1'b0};

            /*
            //New changes for Stage 2
                    `define LDR     6'b001110
                    `define LDR1    6'b001111
                    //              6'b010000
                    `define B1      6'b010000
                    `define CALL    6'b010001
                    //              6'b010001

                    `define HALT    6'b010010

                    `define LDR2    6'b010011
                    `define LDR3    6'b010100
                    `define LDR4    6'b010101
                    `define WRITEC  6'b010000

                    `define STR     6'b010110
                    `define STR1     6'b010111
                    `define STR2     6'b011000
                    `define STR3     6'b011001
                    `define STR4     6'b011010
                    `define STR5     6'b011011



                    //`define XXX         6'b011100
                    `define BX      6'b011101
                    `define BL     6'b011110
                    //`define PCWRITE1    6'b011111

                    `define BX1     6'b100000
                    //`define PCREAD2     6'b100001
                    //`define PCREAD3     6'b100010
                    //`define PCREAD4     6'b100011
                */
//                          {state_next,loada,loadb,loadc,loads,  asel,bsel,vsel,nsel,write,  load_pc,reset_pc,load_addr,load_ir,addr_sel, mem_cmd,isHalt,PCsel} = caseOut;
            {`LDR,3'b011,2'b00}: caseOut = {`LDR1,1'b1,1'b0,1'b0,1'b0,  1'b0,1'b0,2'b00,3'b001,1'b0,  1'b0,1'b0,1'b0,1'b0,1'b0,2'b00, 1'b0,1'b0}; //passed the value to Ain
            {`LDR1,3'b011,2'b00}: caseOut = {`LDR2,1'b0,1'b0,1'b1,1'b1,  1'b0,1'b1,2'b00,3'b000,1'b0,  1'b0,1'b0,1'b1,1'b0,1'b0,2'b10, 1'b0,1'b0}; //datapath out
            {`LDR2,3'b011,2'b00}: caseOut = {`LDR3,1'b0,1'b0,1'b0,1'b0,  1'b0,1'b1,2'b00,3'b000,1'b0,  1'b0,1'b0,1'b1,1'b0,1'b0,2'b10, 1'b0,1'b0}; //read memory address 5//data address: 5
            {`LDR3,3'b011,2'b00}: caseOut = {`LDR4,1'b0,1'b0,1'b0,1'b0,  1'b0,1'b1,2'b00,3'b000,1'b0,  1'b0,1'b0,1'b0,1'b0,1'b0,2'b10, 1'b0,1'b0}; //load ir clock
            {`LDR4,3'b011,2'b00}: caseOut = {`WRITEC,1'b0,1'b0,1'b0,1'b0,  1'b0,1'b0,2'b11,3'b000,1'b0,  1'b0,1'b0,1'b0,1'b0,1'b0,2'b10, 1'b0,1'b0};

            {`WRITEC,3'b011,2'b00}: caseOut = {`SIF1,1'b0,1'b0,1'b0,1'b0,  1'b0,1'b0,2'b11,3'b000,1'b1,  1'b0,1'b0,1'b0,1'b0,1'b0,2'b10, 1'b0,1'b0};

//                          {state_next,loada,loadb,loadc,loads,  asel,bsel,vsel,nsel,write,  load_pc,reset_pc,load_addr,load_ir,addr_sel, mem_cmd,isHalt,PCsel} = caseOut;

            {`STR,3'b100,2'b00}: caseOut =  {`STR1,1'b1,1'b0,1'b0,1'b0,  1'b0,1'b1,2'b00,3'b001,1'b0,  1'b0,1'b0,1'b0, 1'b0, 1'b0, 2'b00, 1'b0,1'b0};
            {`STR1,3'b100,2'b00}: caseOut = {`STR2,1'b0,1'b0,1'b1,1'b1,  1'b0,1'b1,2'b00,3'b000,1'b0,  1'b0,1'b0,1'b0, 1'b0, 1'b0, 2'b00, 1'b0,1'b0}; //indicate write
            {`STR2,3'b100,2'b00}: caseOut = {`STR3,1'b0,1'b1,1'b0,1'b0,  1'b1,1'b0,2'b00,3'b000,1'b0,  1'b0,1'b0,1'b0, 1'b0, 1'b0, 2'b00, 1'b0,1'b0};
            {`STR3,3'b100,2'b00}: caseOut = {`STR4,1'b0,1'b1,1'b1,1'b1,  1'b1,1'b0,2'b00,3'b000,1'b0,  1'b0,1'b0,1'b1, 1'b0, 1'b0, 2'b00, 1'b0,1'b0};
            {`STR4,3'b100,2'b00}: caseOut = {`STR5,1'b0,1'b0,1'b1,1'b1,  1'b1,1'b0,2'b00,3'b000,1'b0,  1'b0,1'b0,1'b0, 1'b0, 1'b0, 2'b01, 1'b0,1'b0};
            {`STR5,3'b100,2'b00}: caseOut = {`SIF1,1'b0,1'b0,1'b0,1'b0,  1'b0,1'b0,2'b00,3'b000,1'b0,  1'b0,1'b0, 1'b0, 1'b0, 1'b0, 2'b01, 1'b0,1'b0};
            //10000 010 001 00000

            {`HALT,3'b111,2'b00}: caseOut = {`SRESET,1'b0,1'b0,1'b0,1'b0,  1'b0,1'b0,2'b00,3'b000,1'b0, 1'b0,1'b0, 1'b0, 1'b0, 1'b0, 2'b00, 1'b1,1'b0};

            //lab8 part1
            {`B1,3'b001,2'bxx}: caseOut = {`SIF1,1'b0,1'b0,1'b0,1'b0,  1'b0,1'b0,2'b00,3'b000,1'b0, 1'b0,1'b0, 1'b0, 1'b0, 1'b0, 2'b00, 1'b0,1'b0};

//                          {state_next,loada,loadb,loadc,loads,  asel,bsel,vsel,nsel,write,  load_pc,reset_pc,load_addr,load_ir,addr_sel, mem_cmd,isHalt,PCsel} = caseOut;
            //   {`BL,3'b010,2'b11}: caseOut = {`BL1,1'b0,1'b0,1'b0,1'b0,  1'b0,1'b0,2'b00,3'b000,1'b0, 1'b0,1'b0, 1'b0, 1'b0, 1'b0, 2'b00, 1'b0,1'b0};
            {`BL,3'b010,2'b11}: caseOut = {`SIF1,1'b0,1'b0,1'b0,1'b0,  1'b0,1'b0,2'b01,3'b001,1'b1,   1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,1'b0,1'b0};
            //
            {`BX,3'b010,2'b00}: caseOut = {`BX1,1'b0,1'b1,1'b0,1'b0,  1'b0,1'b0,2'b00,3'b000,1'b0,   1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,1'b0,1'b0};
            {`BX1,3'b010,2'b00}: caseOut = {`BX2,1'b0,1'b0,1'b1,1'b0,  1'b1,1'b0,2'b00,3'b000,1'b0,   1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,1'b0,1'b1};
            {`BX2,3'b010,2'b00}: caseOut = {`SIF1,1'b0,1'b0,1'b0,1'b0,  1'b0,1'b0,2'b00,3'b000,1'b0,   1'b1,1'b0,1'b0,1'b0,1'b0,2'b00,1'b0,1'b1};
            //{`BX3,3'b010,2'b00}: caseOut = {`SIF1,1'b0,1'b0,1'b1,1'b0,  1'b0,1'b0,2'b00,3'b000,1'b0,   1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,1'b0,1'b1};
            //{`BX4,3'b010,2'b00}: caseOut = {`SIF1,1'b0,1'b0,1'b1,1'b0,  1'b0,1'b0,2'b00,3'b000,1'b0,   1'b0,1'b0,1'b0,1'b0,1'b0,2'b00,1'b0,1'b1};
            //{`PCREAD2,3'b010,2'b10}: caseOut = {`PCREAD3,1'b0,1'b0,1'b0,1'b0,  1'b0,1'b0,2'b00,3'b000,1'b0, 1'b0,1'b0, 1'b0, 1'b0, 1'b0, 2'b00, 1'b0,1'b0};


            /*
            SPECIFICATION:
                nsel:
                    case (nsel)
                    3'b001: {readnum, writenum} = {2{Rn}};
                    3'b010: {readnum, writenum} = {2{Rm}};
                    default: {readnum, writenum} = {2{Rd}};

                vsel:
                    vsel = 2'b11 -> mdata
                    vsel = 2'b10 -> sximm8
                    vsel = 2'b01 -> {8'b0,PC}
                    vsel = 2'b00 -> C

                mem_cmd:
                    `define MREAD   2'b10
                    `define MWRITE  2'b01
            */



            default : caseOut = {`SRESET,1'b0,1'b0,1'b0,1'b0,   1'b0,1'b0,2'b0,3'b000,1'b0,    1'b0,1'b0,1'b0,1'b0,1'b0,2'b00, 1'b0,1'b0};
        endcase
        {state_next,loada,loadb,loadc,loads,   asel,bsel,vsel,nsel,write,   load_pc,reset_pc,load_addr,load_ir,addr_sel,mem_cmd,  isHalt,PCsel} = caseOut;
        //2   3                                                             //2
    end
endmodule 

/*

    `define Branch 3
    `define BSTAYED   3'b011
    `define BRESET  3'b111
    `define BDECODE 3'b110
    `define B      3'b000
    `define BEQ    3'b001
    `define BNE    3'b010
    `define BLT    3'b100
    `define BLE    3'b010


    wire [2:0] present_state, state_next_reset;
    reg [2:0] state_next;

// state DFF for control FSM for exponent circuit
    vDFF #(3) STATE(clk,state_next_reset,present_state);
// reset mux for control FSM for exponent circuit
    assign state_next_reset = reset_pc ? `BRESET : state_next;

    always @(*) begin

        casex({present_state, opcode, op, cond})

            //First cycle checks if state is reset
            {`BRESET, 3'b001, 2'b00, 3'bxxx}:
                begin
                    state_next = `BDECODE;
                    next_pc = 9'b0;
                end

            {`BSTAYED , 3'b001, 2'b00, 3'bxxx}:
                begin
                    state_next = `BDECODE;
                end

            //Decode goes to the next state based on value of cond
            {`BDECODE, 3'b001, 2'b00 , 3'b000}:
                begin
                    state_next = `B;

                end

            {`BDECODE, 3'b001, 2'b00 , 3'b001}:
                begin
                    state_next = `BEQ;

                end

            {`BDECODE, 3'b001, 2'b00 , 3'b010}:
                begin
                    state_next = `BNE;

                end

            {`BDECODE, 3'b001, 2'b00 , 3'b011}:
                begin
                    state_next = `BLT ;

                end

            {`BDECODE, 3'b001, 2'b00 , 3'b100}:
                begin
                    state_next = `BLE ;

                end

            {`B, 3'b001, 2'b00 , 3'b000}:
                begin
                    state_next = `BSTAYED;
                    next_pc = PC + 1 + sximm8[8:0];
                end

            {`BEQ, 3'b001, 2'b00 , 3'b001}:
                begin
                    state_next = `BSTAYED;

                    if(Z==1'b1) begin
                        next_pc = PC + 1 + sximm8[8:0];
                    end

                    else begin
                        next_pc = PC + 1;
                    end
                end
         default: state_next = 3'bxxx;

        endcase


    end

*/
