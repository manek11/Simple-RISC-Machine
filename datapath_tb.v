module datapath_tb();

//using reg for input and wire for output
  reg clk;
  reg [15:0] datapath_in;
  wire [15:0] datapath_out;
  wire Z_out;

//OTHER
  reg vsel;
  reg asel;
  reg bsel;
  reg [1:0] ALUop;
  reg loada;
  reg loadb;
  reg loadc;
  reg loads;

//wires for REGFILE
  reg [2:0] writenum; 
  reg [2:0] readnum;
  reg write;

//wires for SHIFTER
  reg [1:0] shift;

  reg err;

/*
//ADDITIONAL
reg [15:0] data_in;
wire [15:0] data_out;

//ALU
reg [15:0] Ain;
reg [15:0] Bin;
wire [15:0] ALU_out;

//Shifter
reg [15:0] LECA_out;
reg [15:0] LECB_out;
wire [15:0] shifterOut;
*/

//-------------------------------------------------------


  datapath DUT ( .clk         (clk),

                // register operand fetch stage
                .readnum     (readnum),
                .vsel        (vsel),
                .loada       (loada),
                .loadb       (loadb),

                // computation stage (sometimes called "execute")
                .shift       (shift),
                .asel        (asel),
                .bsel        (bsel),
                .ALUop       (ALUop),
                .loadc       (loadc),
                .loads       (loads),

                // set when "writing back" to register file
                .writenum    (writenum),
                .write       (write),  
                .datapath_in (datapath_in),

                // outputs
                .Z_out       (Z_out),
                .datapath_out(datapath_out)

/*
		// ADDITIONAL
		.data_in	(data_in),
		.data_out	(data_out),

		.Ain	(Ain),
		.Bin	(Bin),
		.ALU_out	(ALU_out),

		.LECB_out	(LECB_out),
		.LECA_out	(LECA_out),
		.shifterOut	(shifterOut)
*/

             );

//-------------------------------------------------------
initial begin
  //CLK MODIFY
  clk = 0;
  forever begin
    #5 clk = !clk;
  end
end



initial begin
err=1'b0;


//TEST 1
//initial values
asel =1;
bsel =1;

//write 7 to R0
vsel = 1;
datapath_in = 16'b0000_0000_0000_0111;
#5;
writenum = 3'b000;
write = 1;
#20;

//write 2 to R1
writenum = 3'b001;
datapath_in = 16'b0000_0000_0000_0010;
#20;

/*
//rest the input 
vsel = 0;
write = 0;
#5;
*/


//read 7
shift = 2'b01;
readnum = 3'b000;
loadb = 1;
#20;
loadb=0;

//read 2
readnum = 3'b001;
loada = 1;
#20;
loada=0;

//a and b LEC select read value
asel = 0;
bsel = 0;
#50;

//operate addition of inputs
ALUop = 2'b00;
#5;

//c LEC enabled
//z LEC enabled
loadc = 1;
loads = 1;
#10;
//disabled c and z LEC enabled
loadc = 0;
loads = 0;


#50;

//if the output is 16, then display PASS
if( datapath_out == 16'b0000_0000_0001_0000 ) begin 
$display("PASSED"); 
end else begin 
$display("FAILED"); 
assign err = 1'b1;
end

$stop;
end

endmodule
