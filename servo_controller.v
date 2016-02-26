/*
	servo controller
	
	takes an 8-bit position as an input
	output a single pwm signal with period of ~20ms
		pulse width = 1ms -> 2ms full scale. 1.5ms is center position
	
	
	approximate scaling.
		position = 0 -> 42,240 (should be 50,000)
		position = 255 -> 107,520 (should be 100,000)
	This means that it will probably reach it's limits at either end
		and not quite scale correctly
*/
module servo_controller (
	input clk,
	input rst,
	input [7:0] position,
	output servo
	);
	
	reg pwm_q, pwm_d;
	reg [19:0] ctr_q, ctr_d;

	assign servo = pwm_q;
	
	always @(*) begin
		ctr_d = ctr_q + 1'b1;
		
		if (position + 9'd165 > ctr_q[19:8]) begin
			pwm_d = 1'b1;
		end else begin
			pwm_d = 1'b0;
		end
	end
	
	always @(posedge clk) begin
		if (rst) begin
			ctr_q <= 1'b0;
		end else begin
			ctr_q <= ctr_d;
		end
	
		pwm_q <= pwm_d;
	end	
	
endmodule