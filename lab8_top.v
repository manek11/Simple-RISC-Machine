module lab8_top(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5, CLOCK_50); //CLOCK_50

input [3:0] KEY;
input [9:0] SW;
output wire [9:0] LEDR;
output reg [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;
input CLOCK_50;

wire reset;
wire [15:0] read_data, write_data;
wire [1:0] mem_cmd;
wire [8:0] mem_addr;

wire [15:0] triOut;
assign read_data = triOut;

//assign CLOCK_50 = ~KEY[0];
//wire clk = CLOCK_50;

assign reset = ~KEY[1];
//cpu CPU(clk,reset, read_data, mem_cmd, mem_addr, write_data);

//TODO: added isHalt to lab8_top
wire isHalt;

//.clk(CLOCK_50)



cpu CPU(.clk(CLOCK_50), .reset(~KEY[1]), .read_data(read_data),
 .mem_cmd(mem_cmd), .mem_addr(mem_addr), .write_data(write_data), .isHalt(isHalt));


wire [7:0] read_address;
wire [7:0] write_address;
wire eq9Out, eq8Out, eqWriteOut;

wire [15:0] dout,din;

assign read_address = mem_addr[7:0];
assign write_address = mem_addr[7:0];


`define M       2
`define MREAD   2'b10
`define MWRITE  2'b01
`define MZERO   1'b1
assign eq9Out = (`MREAD == mem_cmd);
assign eqWriteOut = (`MWRITE == mem_cmd);
assign eq8Out = ~(`MZERO == mem_addr[8]);
assign triOut = (eq8Out && eq9Out) ? dout : 16'bz;

assign read_data = triOut;
assign din = write_data;

RAM #(16,8,"Final.txt") MEM(CLOCK_50,read_address,write_address,eqWriteOut,din,dout); //#(16,8,"test.txt"): 16 bits wide data, 2^8 = 256 memory addresses in test.txt

wire [8:0] fixed_in, hex_fix;
wire cout1, cout2;
wire [7:0] in1,in2;
assign in2 = SW[7:0];

    //-----------------LEDR-------------------
    wire [7:0] hex_out;
    //reg LED8;

reg LED8;

	always@(*) begin
	case(isHalt)
	1'b1: LED8 <= 1'b1;
	default: LED8 <=1'b0;
	endcase	
	end

    
	assign LEDR[7:0] = reset ? 8'b0000_0000: hex_out;
	
    //assign LEDR[7:0] = hex_out;
    assign LEDR[8] =reset ? 1'b0 : LED8;


    wire [6:0] DPC1,DPC2, Dwrite_data1, Dwrite_data2, Dwrite_data3, Dwrite_data4, Dread_data1, Dread_data2, Dread_data3, Dread_data4, Dmem_addr1,Dmem_addr2, Dmem_cmd;

wire [3:0] in;
wire [7:0] segs;

assign in = 4'b1010;

sseg hex_layout(in,segs);


Condition1 C1(fixed_in, mem_cmd, mem_addr, in1, in2, cout1, read_data);
Condition2 C2(hex_fix, mem_cmd, mem_addr, cout2);
LEC #(8) L1(CLOCK_50, cout2, write_data[7:0], hex_out);

endmodule 


//--------HELPER MODULES--------//
module RAM(clk,read_address,write_address,write,din,dout);
    parameter data_width = 32;
    parameter addr_width = 4;
    parameter filename = "demo.txt";

    input clk;
    input [addr_width-1:0] read_address, write_address;
    input write;
    input [data_width-1:0] din;
    output [data_width-1:0] dout;
    reg [data_width-1:0] dout;

    reg [data_width-1:0] mem [2**addr_width-1:0];

    initial $readmemb(filename, mem);

    always @ (posedge clk) begin
        if (write)
            mem[write_address] <= din;
        dout <= mem[read_address]; // dout doesn't get din in this clock cycle
        // (this is due to Verilog non-blocking assignment "<=")
    end
endmodule

module sseg(in,segs);
    input [3:0] in;
    output [6:0] segs;

    reg [6:0] segs;
    always@(in) begin
        case (in)
		4'b1111: segs = 	 7'b0101011;
            default: segs =  7'b1111111;
        endcase
    end

endmodule

module Condition1 (fixed_in, mem_cmd1, mem_addr1, in1, in2, cout1, read_data1);
    input [8:0] fixed_in, mem_addr1;
    input [1:0] mem_cmd1;
    output reg cout1;
    output [15:0] read_data1;

    input [7:0] in1,in2;

    wire [7:0] in1;

    always @(*) begin
        case ({mem_addr1, mem_cmd1})
            11'b101000000_10: cout1 = 1'b1; //101000000 //256
            default: cout1 = 1'b0;
        endcase
    end

    wire [7:0] x1,x2;

    assign x1 = cout1 ? in1: 8'bz;
    assign x2 = cout1 ? in2: 8'bz;

    assign read_data1[15:8] = x1;
    assign read_data1[7:0] = x2;
endmodule 

module Condition2( hex_fix, mem_cmd2, mem_addr2,cout2);
    input [8:0] hex_fix;
    input [1:0] mem_cmd2;
    input [8:0] mem_addr2;
    output reg cout2;

    //assign hex_fix = 9'b100000000;

    always @(*)begin
        case({mem_addr2, mem_cmd2})
            11'b100000000_01: cout2 = 1'b1;  //100000000
            default : cout2 = 1'b0;
        endcase
    end
endmodule 

module vDFF(clk,D,Q);
  parameter n=1;
  input clk;
  input [n-1:0] D;
  output [n-1:0] Q;
  reg [n-1:0] Q;
  always @(posedge clk)
    Q <= D;
endmodule
