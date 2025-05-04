module CEL_PROJECT(

	//////////// CLOCK //////////
	input 		          		ADC_CLK_10,
	input 		          		MAX10_CLK1_50,
	input 		          		MAX10_CLK2_50,

	//////////// SEG7 //////////
	output		     [7:0]		HEX0,
	output		     [7:0]		HEX1,
	output		     [7:0]		HEX2,
	output		     [7:0]		HEX3,
	output		     [7:0]		HEX4,
	output		     [7:0]		HEX5,

	//////////// KEY //////////
	input 		     [1:0]		KEY,

	//////////// LED //////////
	output		     [9:0]		LEDR,

	//////////// SW //////////
	input 		     [9:0]		SW,

	//////////// GPIO, GPIO connect to GPIO Default //////////
	inout 		    [35:0]		GPIO,
	
		//////////// VGA //////////
	output reg		     [3:0]		VGA_B,
	output reg		     [3:0]		VGA_G,
	output reg		          		VGA_HS,
	output reg		     [3:0]		VGA_R,
	output reg		          		VGA_VS
);


wire [31:0] my_PC, my_rd;

assign clk = ~KEY[1]; //ADC_CLK_10;

my_computer U1(.clk(clk), .reset(~KEY[0]), .my_sw(SW[7:0]), .my_rd(my_rd), .my_PC(my_PC));		// Excluding comments 200+ lines of "code"

assign LEDR = SW[9] ? my_PC : my_rd;

wire [31:0]    col, row;
wire [3:0]     red, green, blue;

/*
assign red     = SW[0] ? 4'hf : 4'h0; 
assign green   = SW[1] ? 4'hf : 4'h0;
assign blue    = SW[2] ? 4'hf : 4'h0;
*/

wire           h_sync, v_sync;
wire           disp_ena;
wire           vga_clk;

pll vgapll_inst (
    .inclk0    (MAX10_CLK1_50),
    .c0        (vga_clk)
    );

always @(posedge vga_clk) begin
   if (disp_ena == 1'b1) begin
      VGA_R <= SW[0] ? 4'hf : 4'h0; // red
      VGA_B <= SW[2] ? 4'hf : 4'h0; // blue
      VGA_G <= SW[1] ? 4'hf : 4'h0; // green
   end else begin
      VGA_R <= 4'd0;
      VGA_B <= 4'd0;
      VGA_G <= 4'd0;
   end
   VGA_HS <= h_sync;
   VGA_VS <= v_sync;
end

vga_controller control (
   .pixel_clk  (vga_clk),
   .reset_n    (KEY[0]),
   .h_sync     (h_sync),
   .v_sync     (v_sync),
   .disp_ena   (disp_ena),
   .column     (col),
   .row        (row)
   );

endmodule
