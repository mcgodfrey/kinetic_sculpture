module buffer_ctrl #(parameter NUM_SERVOS=1) (
		input clk,
		input rst,
		input [5:0] servo_num,
		input [7:0] servo_pos,
		input new_pos,
		output [8*NUM_SERVOS-1:0] pos_buffer
	);
	
	reg [8*NUM_SERVOS-1:0] pos_buffer_d, pos_buffer_q;
	assign pos_buffer = pos_buffer_q;
	
	always @(*) begin
		pos_buffer_d = pos_buffer_q;
		if (new_pos) begin
			//pos_buffer_d[servo_num<<4+7:servo_num<<4+0] = servo_pos;
			pos_buffer_d[((servo_num<<4)+7) -: 8] = servo_pos;
		end
	end
	
	always @(posedge clk) begin
		pos_buffer_q <= pos_buffer_d;
	
	end
		
	
endmodule