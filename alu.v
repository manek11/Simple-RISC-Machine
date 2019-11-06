module ALU(Ain,Bin,ALUop,out,Z);
input [15:0] Ain, Bin;
input [1:0] ALUop;
output [15:0] out;
output [2:0] Z;

//Z[0]: representing status as in lab5
//Z[1]: representing "negative flag" MSB of out = 1, then set to 1, otherwise 0 
//Z[2]: represent overflow tag 

reg [15:0] out;
reg [2:0] Z;

reg subs;
wire overflow;
wire [15:0]ss;

always @(*) begin
case(ALUop)
2'b00: subs = 1'b0;
2'b01: subs =1'b1;
2'b10: subs = 1'b0;
2'b11: subs = 1'b0;
default: subs= 1'bx;
endcase
end

AddSub #(16) Main(Ain, Bin, subs, ss ,overflow);

always @(*) begin
case(ALUop)
2'b00: out =ss;
2'b01: out =ss;
2'b10: out = Ain & Bin;
2'b11 : out = ~Bin;
default: out = 16'bxxxx_xxxx_xxxx_xxxx;
endcase
end 

always @(*) begin
if (out == 16'b0000_0000_0000_0000) begin
Z[0] = 1'b1;
end
else begin
Z[0] = 1'b0;
end

if(out[15]) begin
Z[1] =1'b1;
end
else begin
Z[1] = 1'b0;
end

if(overflow == 1'b1) begin
Z[2] = 1'b1;
end
else begin
Z[2] = 1'b0;
end
end
endmodule


//Full adder to account overflow
module AddSub(a,b,sub,s,ovf) ;
  parameter n = 8 ;
  input [n-1:0] a, b ;
  input sub ;           // subtract if sub=1, otherwise add
  output [n-1:0] s ;
  output ovf ;          // 1 if overflow
  wire c1, c2 ;         // carry out of last two bits
  wire ovf = c1 ^ c2 ;  // overflow if signs don't match

  // add non sign bits
  Adder1 #(n-1) ai(a[n-2:0],b[n-2:0]^{n-1{sub}},sub,c1,s[n-2:0]) ;
  // add sign bits
  Adder1 #(1)   as(a[n-1],b[n-1]^sub,c1,c2,s[n-1]) ;
endmodule


//Multi Bit adder
module Adder1(a,b,cin,cout,s) ;
  parameter n = 8 ;
  input [n-1:0] a, b ;
  input cin ;
  output [n-1:0] s ;
  output cout ;
  wire [n-1:0] s;
  wire cout ;

  assign {cout, s} = a + b + cin ;
endmodule 
