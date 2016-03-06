/*
	8 bit up-down counter.
	Based on code from Mojo tutorial 
	https://embeddedmicro.com/tutorials/mojo/pulse-width-modulation
	   /\      /\
	  /  \    /  \
	 /    \  /    \
	/      \/      \
	the CRT_LEN determines the period of the resulting counter
	26 results in ~1s period.
*/
module counter #(parameter CTR_LEN = 27) (
	input wire clk,
	input wire rst,
	output reg [7:0] value
	);
   
	//flip flop registers
	reg[CTR_LEN-1:0] ctr_d, ctr_q;
	
	//We are creating a count-up/count-down timer here with a 
	//  triangular shape.
	//The MSB determines the direction (up or down)
	//The 8 next highest bits form out output value
	//The remaining bits are not used (they are needed to get the timing
	//  right though, otherwise the counter would be too fast)
	//There is a little trick if inverting the value to get 
	//   it to count down instead of up when the MSB == 1
	always @(ctr_q) begin
		ctr_d = ctr_q + 1'b1;
     
		if (ctr_q[CTR_LEN-1] == 1)
			value = ~ctr_q[CTR_LEN-2:CTR_LEN-9];
		else
			value = ctr_q[CTR_LEN-2:CTR_LEN-9];
	end
	
	//standard positive edge of clk triggered always block
	//reset sets the coutner to zero
	//otherwise it sets the output (ctr_q) to the input (crt_d)
	always @(posedge clk) begin
		if (rst == 1) begin
			ctr_q <= 'b0;
		
		end else begin
			ctr_q <= ctr_d;
		end
		
	end
   
endmodule