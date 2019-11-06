module regfile_tb ();

reg [15:0] sim_data_in;
reg [2:0] sim_writenum, sim_readnum;
reg sim_write, sim_clk;
reg err; //Self-ERROR-checking
wire [15:0] sim_data_out;

regfile DUT(
	.data_in(sim_data_in),
	.writenum(sim_writenum),
	.readnum(sim_readnum),
	.write(sim_write),
	.clk(sim_clk),
	.data_out(sim_data_out)
);


initial begin
  //CLK MODIFY
sim_readnum = 3'b001;
sim_data_in = 16'b0000_0000_0000_0000;
sim_writenum = 3'b000;
sim_write = 1;
err = 1'b0;

  sim_clk = 0;
  forever begin
    sim_clk = !sim_clk;
    #5; 
  end
end

initial begin
//INITIALIZE

//TEST 1 
//Loading 42 into R1
sim_data_in = 16'b0000_0000_0010_1010;
sim_writenum = 3'b001;
sim_write = 1;
#15;
sim_readnum = 3'b001;
#15;
if(sim_data_out !== 16'b0000_0000_0010_1010) begin
  $display("ERROR ** output is %b, expected %b", sim_data_out, 16'b0000_0000_0010_1010);
  err = 1'b1;
end


//TEST 2
//Loading 36085 to R2
sim_data_in = 16'b1000_1100_1111_1010;
sim_writenum = 3'b010;
sim_write = 1;
#15;
sim_readnum = 3'b010;
#5;
if(sim_data_out !== 16'b1000_1100_1111_1010) begin
  $display("ERROR ** output is %b, expected %b", sim_data_out, 16'b1000_1100_1111_1010);
  err = 1'b1;
end


//TEST 3
//Loading 128 to R3
sim_data_in = 16'b0000_0000_1000_0000;
sim_writenum = 3'b011;
sim_write = 1;
#15;
sim_readnum = 3'b011;
#5;
if(sim_data_out !== 16'b0000_0000_1000_0000) begin
  $display("ERROR ** output is %b, expected %b", sim_data_out, 16'b0000_0000_1000_0000);
  err = 1'b1;
end


//TEST 4
//Loading 256 to R4
sim_data_in = 16'b0000_0010_0000_0000;
sim_writenum = 3'b100;
sim_write = 1;
#15;
sim_readnum = 3'b100;
#5;
if(sim_data_out !== 16'b0000_0010_0000_0000) begin
  $display("ERROR ** output is %b, expected %b", sim_data_out, 16'b0000_0010_0000_0000);
  err = 1'b1;
end


//TEST 5
//Loading 512 to R5
sim_data_in = 16'b0000_1000_0000_0000;
sim_writenum = 3'b101;
sim_write = 1;
#15;
sim_readnum = 3'b101;
#5;
if(sim_data_out !== 16'b0000_1000_0000_0000) begin
  $display("ERROR ** output is %b, expected %b", sim_data_out, 16'b0000_1000_0000_0000);
  err = 1'b1;
end


//TEST 6
//Loading 1024 to R6
sim_data_in = 16'b0010_0000_0000_0000;
sim_writenum = 3'b110;
sim_write = 1;
#15;
sim_readnum = 3'b110;
#5;
if(sim_data_out !== 16'b0010_0000_0000_0000) begin
  $display("ERROR ** output is %b, expected %b", sim_data_out, 16'b0010_0000_0000_0000);
  err = 1'b1;
end


//TEST 7
//Loading 2048 to R7
sim_data_in = 16'b1000_0000_0000_0000;
sim_writenum = 3'b111;
sim_write = 1;
#15;
sim_readnum = 3'b111;
#5;
if(sim_data_out !== 16'b1000_0000_0000_0000) begin
  $display("ERROR ** output is %b, expected %b", sim_data_out, 16'b1000_0000_0000_0000);
  err = 1'b1;
end


//TEST 8
//Loading 0 to R0
sim_data_in = 16'b0000_0000_0000_0000;
sim_writenum = 3'b000;
sim_write = 1;
#15;
sim_readnum = 3'b000;
#500;
if(sim_data_out !== 16'b0000_0000_0000_0000) begin
  $display("ERROR ** output is %b, expected %b", sim_data_out, 16'b0000_0000_0000_0000);
  err = 1'b1;
end




if( err == 1'b0 ) begin $display("PASSED"); end
else begin $display("FAILED"); end

$stop;
end// TESTS

endmodule//MODULE
