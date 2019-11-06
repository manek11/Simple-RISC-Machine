//Testbench for shifter module

module shifter_tb ();
reg [15:0] sim_in;
reg [1:0] sim_shift;
wire [15:0] sim_sout;

reg err;

shifter DUT(
.in(sim_in),
.shift(sim_shift),
.sout(sim_sout)
);

initial begin

//make error 0 initially

err = 1'b0;

//Start testbench with all 0's input
sim_in = 16'b0000_0000_0000_0000;
sim_shift = 2'b00;
#50;


//Case for 61647

//Same output
$display("Runnning test 1");

sim_in = 16'b1111_0000_1100_1111;
sim_shift = 2'b00;

#50;

if(sim_sout != 16'b1111_0000_1100_1111) begin
$display("Error! Output is %b, expected is 1111000011001111", sim_sout);
err = 1'b1;
end


#50;

//Output shifted to left
$display("Runnning test 2");

sim_in = 16'b1111_0000_1100_1111;
sim_shift = 2'b01;

#50;

if(sim_sout != 16'b1110_0001_1001_1110) begin
$display("Error! Output is %b, expected is 1110000110011110 ", sim_sout);
err = 1'b1;
end

#50;

//Output shifted to right
$display("Runnning test 3");

sim_in = 16'b1111_0000_1100_1111;
sim_shift = 2'b10;

#100;

if(sim_sout != 16'b0111_1000_0110_0111) begin
$display("Error! Output is %b, expected is 0111100001100111", sim_sout);
err = 1'b1;
end

#100;

//Output shifted to right MSB is copy of B[15]
$display("Runnning test 4");

sim_in = 16'b1111_0000_1100_1111;
sim_shift = 2'b11;

#100;

if(sim_sout != 16'b1111_1000_0110_0111) begin
$display("Error! Output is %b, expected is 1111100001100111", sim_sout);
err = 1'b1;
end

//////////////////////////////


//Case for number 22879

//Output shifted to left
$display("Runnning test 5");

sim_in = 16'b0111_0000_1100_1111;
sim_shift = 2'b00;

#100;

if(sim_sout != 16'b0111_0000_1100_1111) begin
$display("Error! Output is %b, expected is 0111000011001111", sim_sout);
err = 1'b1;
end


#100;

//Output shifted to left

$display("Runnning test 6");

sim_in = 16'b0111_0000_1100_1111;
sim_shift = 2'b01;

#50;

if(sim_sout != 16'b111_0000_1100_11110) begin
$display("Error! Output is %b, expected is 111_0000_1100_11110 ", sim_sout);
err = 1'b1;
end

#50;



//Output shifted to right MSB is copy of B[15]
$display("Runnning test 7");

sim_in = 16'b1111_0000_1100_1111;
sim_shift = 2'b11;

#50;

if(sim_sout != 16'b1111_1000_0110_0111) begin
$display("Error! Output is %b, expected is 1111100001100111", sim_sout);
err = 1'b1;
end
/////////////////////////////


//conditions for error

if(err==1'b0) begin
$display("Passed! All tests successful");
end
else begin
$display("ERROR! Check tests again");
end

$stop;

end
endmodule





