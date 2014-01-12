// This is a test fixture for MIPS.v

module MIPStestfixture;

reg Clock, Resetn;
MIPS CPU(Clock, Resetn);

// Generate a Clock
initial
     Clock = 1'b0;
always #5 Clock = ~Clock;

// Now run it
initial
     begin
     #0  $dumpfile("waves.vcd");
         $dumpvars;

	Resetn = 1'b0;
	 
     #10 Resetn = 1'b1;
          
     
     #900 $finish;
     end

endmodule     
     
	
