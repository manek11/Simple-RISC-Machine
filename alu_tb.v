//testbench for ALU
module ALU_tb ();
//using reg for inputs and wire for outputs
reg [15:0] sim_Ain, sim_Bin;
reg [1:0] sim_ALUop;
wire [15:0] sim_out;
wire [2:0] sim_Z;

reg err;

//declaring dut
ALU DUT(
.Ain(sim_Ain),
.Bin(sim_Bin),
.ALUop(sim_ALUop),
.out(sim_out),
.Z(sim_Z)
);


initial begin

//initial conditions for all zeros and error = 0
err=1'b0;


sim_Ain = 16'b0000_0000_0000_0000;
sim_Bin = 16'b0000_0000_0000_0000;
sim_ALUop = 2'b00;
#100;

// test for Add
$display("Running test 1");

sim_Ain = 16'b0101_1100_0100_0101;
sim_Bin = 16'b0001_1111_1100_0101;
sim_ALUop = 2'b00;
#100;

if(sim_out!== 16'b0111110000001010) begin
$display("Error! Output is %b, expected is 0111110000001010", sim_out);
err=1'b1;
end


// test for Subtract
$display("Running test 2");

sim_Ain = 16'b0101_1100_0100_0101; //23621
sim_Bin = 16'b0001_1111_1100_0101; //8133
sim_ALUop = 2'b01;
#100

if(sim_out!== 16'b011110010000000) begin
$display("Error! Output is %b, expected is 011110010000000", sim_out);
err=1'b1;
end

// test for AND

$display("Running test 3");

sim_Ain = 16'b0101_1100_0100_0101;
sim_Bin = 16'b0001_1111_1100_0101;
sim_ALUop = 2'b10;
#100

if(sim_out!== 16'b0001110001000101) begin
$display("Error! Output is %b, expected is 0001110001000101", sim_out);
err=1'b1;
end

// test for ~Bin
$display("Running test 4");

sim_Ain = 16'b0101_1100_0100_0101;
sim_Bin = 16'b0001_1111_1100_0101;
sim_ALUop = 2'b11;
#105;

if(sim_out!== 16'b1110000000111010) begin
$display("Error! Output is %b, expected is 1110000000111010", sim_out);
err=1'b1;
end

//----------------------------------------------------------------------

//----------------------------------------------------------------------
if(err==1'b0) begin
$display("No Error. Test Successful");
end

$stop;

#5;

end
endmodule