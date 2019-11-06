module regfile(data_in,writenum,write,readnum,clk,data_out);
input [15:0] data_in;
input [2:0] writenum, readnum;
input write, clk;
output [15:0] data_out;

//fill out the rest
//reg [15:0] register[7:0];
//reg [15:0] outdata;

//wire vDFFtoMuxOut;
wire[7:0] DecOut;
wire[7:0] DecANDWriteOut;
//wire[15:0] RegisterInternal[7:0];//changed
//wire[15:0] RegisterLoop[7:0];

wire[15:0] RegisterOut[7:0];

wire[15:0] R0;
wire[15:0] R1;
wire[15:0] R2;
wire[15:0] R3;
wire[15:0] R4;
wire[15:0] R5;
wire[15:0] R6;
wire[15:0] R7;

wire[7:0] DectoMuxOut;

//8 AND operation
assign DecANDWriteOut[0] = DecOut[0] & write;
assign DecANDWriteOut[1] = DecOut[1] & write;
assign DecANDWriteOut[2] = DecOut[2] & write;
assign DecANDWriteOut[3] = DecOut[3] & write;
assign DecANDWriteOut[4] = DecOut[4] & write;
assign DecANDWriteOut[5] = DecOut[5] & write;
assign DecANDWriteOut[6] = DecOut[6] & write;
assign DecANDWriteOut[7] = DecOut[7] & write;

//decoder instantiation
Dec #(3,8) U1(writenum, DecOut);
Dec #(3,8) U2(readnum, DectoMuxOut);

//register instantiation
//R0
LECreg DFF0(clk,DecANDWriteOut[0],data_in,R0);
//R1
LECreg DFF1(clk,DecANDWriteOut[1],data_in,R1);
//R2
LECreg DFF2(clk,DecANDWriteOut[2],data_in,R2);
//R3
LECreg DFF3(clk,DecANDWriteOut[3],data_in,R3);
//R4
LECreg DFF4(clk,DecANDWriteOut[4],data_in,R4);
//R5
LECreg DFF5(clk,DecANDWriteOut[5],data_in,R5);
//R6
LECreg DFF6(clk,DecANDWriteOut[6],data_in,R6);
//R7
//Trying out a module
LECreg DFF7(clk,DecANDWriteOut[7],data_in,R7);

//LEC DFF7(clk,DecANDWriteOut[7],data_in,RegisterOut[7]);

//MuxOut
reg [15:0] out;
always @(*) begin
if(DectoMuxOut[0] == 1'b1) begin
out = R0;
end else 
if(DectoMuxOut[1] == 1'b1) begin
out = R1;
end else
if(DectoMuxOut[2] == 1'b1) begin 
out = R2;
end else
if(DectoMuxOut[3] == 1'b1) begin
out = R3;
end else
if(DectoMuxOut[4] == 1'b1) begin
out = R4;
end else
if(DectoMuxOut[5] == 1'b1) begin
out = R5;
end else
if(DectoMuxOut[6] == 1'b1) begin 
out = R6;
end else
if(DectoMuxOut[7] == 1'b1) begin
out = R7;
end else
out = 16'b0000_0000_0000_0000;
end
assign data_out = out;
/*
if(DectoMuxOut[0] == 1'b1) begin
out = RegisterOut[0];
end else 
*/

endmodule// main module end

/*
module Mux8_16(R7, R6, R5, R4, R3, R2, R1, R0, s, b);
	input [15:0] R7, R6, R5, R4, R3, R2, R1, R0; //input
	input [7:0] s;
	output [15:0] b;
	assign b = 
({s[0],s[0],s[0],s[0],s[0],s[0],s[0],s[0],s[0],s[0],s[0],s[0],s[0],s[0],s[0],s[0]} & R0)|
({s[1],s[1],s[1],s[1],s[1],s[1],s[1],s[1],s[1],s[1],s[1],s[1],s[1],s[1],s[1],s[1]} & R1)|
({s[2],s[2],s[2],s[2],s[2],s[2],s[2],s[2],s[2],s[2],s[2],s[2],s[2],s[2],s[2],s[2]} & R2)|
({s[3],s[3],s[3],s[3],s[3],s[3],s[3],s[3],s[3],s[3],s[3],s[3],s[3],s[3],s[3],s[3]} & R3)|
({s[4],s[4],s[4],s[4],s[4],s[4],s[4],s[4],s[4],s[4],s[4],s[4],s[4],s[4],s[4],s[4]} & R4)|
({s[5],s[5],s[5],s[5],s[5],s[5],s[5],s[5],s[5],s[5],s[5],s[5],s[5],s[5],s[5],s[5]} & R5)|
({s[6],s[6],s[6],s[6],s[6],s[6],s[6],s[6],s[6],s[6],s[6],s[6],s[6],s[6],s[6],s[6]} & R6)|
({s[7],s[7],s[7],s[7],s[7],s[7],s[7],s[7],s[7],s[7],s[7],s[7],s[7],s[7],s[7],s[7]} & R7);
endmodule
*/

module LECreg (clk,load,in,out);
  //parameter n = 8;//which LEC they want to use from 0~7 (pre-condition)
  //parameter bits = 16;//how many bits of input and output

  input clk;
  input load;
  input[15:0] in;
  output[15:0] out;

  wire load;
  wire[15:0] RegisterInternal;//changed
  wire[15:0] RegisterLoop;
  wire[15:0] RegisterOut;

  assign RegisterInternal = load ? in : RegisterOut;
  vDFF #(16) STATE(clk,RegisterInternal,RegisterOut);
  assign out = RegisterOut;

endmodule


//decoder module
module Dec(a, b) ;
  parameter n=2 ;
  parameter m=4 ;

  input  [n-1:0] a ;
  output [m-1:0] b ;

  wire [m-1:0] b = 1 << a ;
endmodule

/*
//register module(16 bits)
module vDFF(clk, in, out) ;
  parameter n = 16;  // width
  input clk ;
  input [n-1:0] in ;
  output [n-1:0] out ;
  reg [n-1:0] out ;

  always @(posedge clk)
    out = in ;
endmodule 

*/