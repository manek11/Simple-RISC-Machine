module cpu_tb;
  reg clk, reset, s, load;
  reg [15:0] in;
  wire [15:0] out;
  wire N,V,Z,w;

  reg err;

  cpu DUT(clk,reset,s,load,in,out,N,V,Z,w);

  initial begin
    clk = 0; #5;
    forever begin
      clk = 1; #5;
      clk = 0; #5;
    end
  end

/*
//Test 1
MOV R0, #7 // this means, take the absolute number
7 and store it in R0
MOV R1, #2 // this means, take the absolute number 
2 and store it in R1
ADD R2, R1, R0, LSL#1 // this means 
R2 = R1 + (R0 shifted left by 1) = 2+14=16

in = 16'b1101_0000_0000_0111;

in = 16'b1101_0001_0000_0010;

in = 16'b1010_0001_0100_1000;
*/

  initial begin
    err = 0;
    reset = 1; s = 0; load = 0; in = 16'b0;
    #10;
    reset = 0; 
    #10;

    in = 16'b1101000000000111;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.DP.REGFILE.R0 !== 16'h7) begin
      err = 1;
      $display("FAILED: MOV R0, #7");
      $stop;
    end

    in = 16'b1101000100000010;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.DP.REGFILE.R1 !== 16'h2) begin
      err = 1;
      $display("FAILED: MOV R1, #2");
      $stop;
    end

    in = 16'b1010000101001000;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.DP.REGFILE.R2 !== 16'h10) begin
      err = 1;
      $display("FAILED: ADD R2, R1, R0, LSL#1");
      $stop;
    end


    //-----------------------TEST2------------------------
    //Testing ADD R2, R1, R0, LSL#1
    reset = 1; s = 0; load = 0; in = 16'b0;
    #10;
    reset = 0; 
    #10;

    in = 16'b1101000000000100;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.DP.REGFILE.R0 !== 16'h4) begin
      err = 1;
      $display("FAILED: MOV R0, #7");
      $stop;
    end

    in = 16'b1101000100000010;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.DP.REGFILE.R1 !== 16'h2) begin
      err = 1;
      $display("FAILED: MOV R1, #2");
      $stop;
    end

    in = 16'b1010000101001000;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.DP.REGFILE.R2 !== 16'hA) begin
      err = 1;
      $display("FAILED: ADD R2, R1, R0, LSL#1");
      $stop;
    end



    //-----------------------TEST3------------------------
    //Testing MOV Rd,Rm{,<sh_op>}

    reset = 1; s = 0; load = 0; in = 16'b0;
    #10;
    reset = 0; 
    #10;

    in = 16'b1100_0000_0000_0010;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.DP.REGFILE.R0 !== 16'hA) begin
      err = 1;
      $display("FAILED: MOV R0,R2{00}");
      $stop;
    end

    in = 16'b1101_0001_0000_0010;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.DP.REGFILE.R1 !== 16'h2) begin
      err = 1;
      $display("FAILED: MOV R1, #2");
      $stop;
    end

    in = 16'b1010_0001_0100_1000;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.DP.REGFILE.R2 !== 16'h16) begin
      err = 1;
      $display("FAILED: ADD R2, R1, R0, LSL#1");
      $stop;
    end

    if (~err) $display("TEST3: MOV Rd,Rm{,<sh_op>} PASSED");


    //-----------------------TEST4------------------------
    //Testing CMP Rn,Rm{,<sh_op>}

    err = 0;
    reset = 1; s = 0; load = 0; in = 16'b0;
    #10;
    reset = 0; 
    #10;

    in = 16'b1101000000000100;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.DP.REGFILE.R0 !== 16'h4) begin
      err = 1;
      $display("FAILED: MOV R0, #4"); //changed to 4
      $stop;
    end

    in = 16'b1101_0001_0000_0010;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.DP.REGFILE.R1 !== 16'h2) begin
      err = 1;
      $display("FAILED: MOV R1, #2");
      $stop;
    end

    in = 16'b1010_1000_0000_0001;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.Z !== 1'b0) begin
      err = 1;
      $display("FAILED: Z!=0 detected");
      $stop;
    end
    if (cpu_tb.DUT.N !== 1'b0) begin
      err = 1;
      $display("FAILED: N!=0 detected");
      $stop;
    end
    if (cpu_tb.DUT.V !== 1'b0) begin
      err = 1;
      $display("FAILED: V!=0 detected");
      $stop;
    end

    if (~err) $display("TEST4: CMP Rn,Rm{,<sh_op>} PASSED");

    //-----------------------TEST5------------------------
    //Testing AND Rd,Rn,Rm{,<sh_op>}

    err = 0;
    reset = 1; s = 0; load = 0; in = 16'b0;
    #10;
    reset = 0; 
    #10;

    in = 16'b1101000000000100;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.DP.REGFILE.R0 !== 16'h4) begin
      err = 1;
      $display("FAILED: MOV R0, #4"); //changed to 4
      $stop;
    end

    in = 16'b1101_0001_0000_0010;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.DP.REGFILE.R1 !== 16'h2) begin
      err = 1;
      $display("FAILED: MOV R1, #2");
      $stop;
    end

    in = 16'b1011_0000_0100_0001;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.DP.REGFILE.R2 !== 16'h0) begin
      err = 1;
      $display("FAILED: AND Rd,Rn,Rm{00}");
      $stop;
    end

    if (~err) $display("TEST5: AND Rd,Rn,Rm{,<sh_op>} PASSED");


    //-----------------------TEST6------------------------
    //Testing MVN Rd,Rm{,<sh_op>}

    err = 0;
    reset = 1; s = 0; load = 0; in = 16'b0;
    #10;
    reset = 0; 
    #10;

    in = 16'b1101000000000100;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.DP.REGFILE.R0 !== 16'h4) begin
      err = 1;
      $display("FAILED: MOV R0, #4"); //changed to 4
      $stop;
    end

    in = 16'b1101_0001_0000_0010;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.DP.REGFILE.R1 !== 16'h2) begin
      err = 1;
      $display("FAILED: MOV R1, #2");
      $stop;
    end

    in = 16'b1011_1000_0100_0000;
    load = 1;
    #10;
    load = 0;
    s = 1;
    #10
    s = 0;
    @(posedge w); // wait for w to go high again
    #10;
    if (cpu_tb.DUT.DP.REGFILE.R2 !== 16'hFFFB) begin
      err = 1;
      $display("FAILED: MVN Rd,Rm{00}");
      $stop;
    end

    if (~err) $display("TEST6: MVN Rd,Rm{,<sh_op>} PASSED");




    if (~err) $display("PASSED");
    $stop;
  end
endmodule
