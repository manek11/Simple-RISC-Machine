module datapath(
  input clk,
  //input [15:0] datapath_in,

  input [15:0] mdata,
  input [15:0] sximm8,
  input [8:0] PC, 
  input [15:0] sximm5, 

  output [15:0] datapath_out,

  output [2:0] Z_out, 

//OTHER
  input [1:0] vsel,
  input asel,
  input bsel,

  input [1:0] ALUop,

  input loada,
  input loadb,
  input loadc,
  input loads,

//REGFILE
  input [2:0] writenum, 
  input [2:0] readnum,
  input write,

//SHIFTER
  input [1:0] shift
);

wire [15:0] datapathLoop;
wire [15:0] muxToReg; //data_in
//wire [2:0] Z_out;

//FIRST MUX (9)
//Update this MUX   
//assign muxToReg = vsel ? datapath_in : datapathLoop;
//Changed

Muxonehot #(16) DataMux(mdata, sximm8, {7'b0, PC},  datapathLoop, vsel, muxToReg);



//REGFILE
wire [15:0] regToLEC; //data_reg
regfile REGFILE(muxToReg,writenum,write,readnum,clk,regToLEC); // regfile.v


wire [15:0] LECA_out;
wire [15:0] LECBToShifter;

LEC #(16) LOADA(clk,loada,regToLEC,LECA_out); // load enabled circuit A
LEC #(16) LOADB(clk,loadb,regToLEC,LECBToShifter); // load enabled circuit B



//INPUT: LECB_out
wire [15:0] shifterOut;
shifter U1(LECBToShifter,shift,shifterOut);


//MUX
wire [15:0] Ain;
wire [15:0] Bin;

assign Bin = bsel ? sximm5 : shifterOut;
assign Ain = asel ? 16'b00000_00000_00000_0 : LECA_out;


//MODIFY
//ALU
wire [15:0] ALU_out;
wire [2:0] Z;

ALU U2(Ain,Bin,ALUop,ALU_out,Z);


//ZSTATUS
//ZSTAT #(3) ZSTATUS(clk,loads,Z,Z_out);

//////////////////////////////////////////



LEC #(3) ZSTATUS(clk, loads, Z, Z_out);



//////////////////////////////////////////

//LEC C
LEC #(16) LOADC(clk,loadc,ALU_out,datapathLoop);
assign datapath_out = datapathLoop;

endmodule


////////////////////////////////////////////////////////

module LEC (clk,load,in,out);
  parameter n = 16;

  input clk;
  input load;
  input[n-1:0] in;
  output[n-1:0] out;
  
  wire load;
  wire[n-1:0] RegisterInternal;//changed
  wire[n-1:0] RegisterLoop;
  wire[n-1:0] RegisterOut;
  
  assign RegisterInternal = load ? in : RegisterOut;
  vDFF #(n) STATE(clk,RegisterInternal,RegisterOut);
  assign out = RegisterOut;

endmodule

module Muxonehot (a3, a2, a1, a0, BinarySelect, MuxonehotOut) ;
  parameter k = 1 ;
  input [k-1:0] a0, a1, a2, a3 ;  // inputs
  input [1:0] BinarySelect ; 

  wire [3:0] s_hot; //hotcode 

  output[k-1:0] MuxonehotOut ;
  
  Dec #(2,4) DEC1(BinarySelect,s_hot);
  Mux3 #(k) MUX1(a3,a2,a1,a0,s_hot,MuxonehotOut);
endmodule


//------------HELPER MODULES

module Mux3(a3, a2, a1, a0, s, b) ;
  parameter k = 1 ;
  input [k-1:0] a0, a1, a2, a3 ;  // inputs
  input [3:0]   s ; // one-hot select
  output[k-1:0] b ;
  wire [k-1:0] b = ({k{s[0]}} & a0) | 
                   ({k{s[1]}} & a1) |
                   ({k{s[2]}} & a2) |
                   ({k{s[3]}} & a3) ;
endmodule



/*
module ZSTAT (clk,load,in,out);
  parameter n = 3;

  input clk;
  input load;
  input[n-1:0] in;
  output[n-1:0] out;

  reg[n-1:0] out;
  wire[n-1:0] next_out;
  
  assign next_out = load ? in : out;
  
  always@(posedge clk)
    out = next_out;

endmodule
*/

