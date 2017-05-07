`default_nettype none

///////////////////////////////////////////////////////////////////////////////
//
// 6.111 FPGA Labkit -- Template Toplevel Module
//
// For Labkit Revision 004
// Created: October 31, 2004, from revision 003 file
// Author: Nathan Ickes, 6.111 staff
// 
// Modified by Robert Sikora May 2017
// to be compatible with Numato Lab MimasV2
//
///////////////////////////////////////////////////////////////////////////////

module lab4(
  // Remove comment from any signals you use in your design!

  // AC97
  output wire /*beep,*/ audio_reset_b, ac97_synch, ac97_sdata_out,
  input wire ac97_bit_clock, ac97_sdata_in,
	
  // BUTTONS, SWITCHES, LEDS
  input wire vol_button_down,
  input wire vol_button_up,
  input wire filt_down,
  input wire filt_up,
  input wire step_button_down,
  input wire step_button_up,
  output wire [7:0] led,
  output wire [7:0] SevenSegment, //FD
  output wire [2:0] SevenSegmentEnable,
  output wire [7:1] IO_P9,

  // CLOCKS
  input wire CLK_100MHz
);
   ////////////////////////////////////////////////////////////////////////////
   //
   // Reset Generation
   //
   // A shift register primitive is used to generate an active-high reset
   // signal that remains high for 16 clock cycles after configuration finishes
   // and the FPGA's internal clocks begin toggling.
   //
   ////////////////////////////////////////////////////////////////////////////
   wire reset;
   SRL16 #(.INIT(16'hFFFF)) reset_sr(.D(1'b0), .CLK(CLK_100MHz), .Q(reset),
                                     .A0(1'b1), .A1(1'b1), .A2(1'b1), .A3(1'b1));
			    
   wire [7:0] from_ac97_data, to_ac97_data;
   wire ready;
	
	   // AC97 driver
   lab4audio a(CLK_100MHz, reset, volume, from_ac97_data, to_ac97_data, ready,
	       audio_reset_b, ac97_sdata_out, ac97_sdata_in,
	       ac97_synch, ac97_bit_clock);
			 
	 // FilterControlModule
   FilterControlModule r(.clock(CLK_100MHz), .reset(reset), .ready(ready),
              .step(step), .filtnum(filtnum),
              .from_ac97_data(from_ac97_data),.to_ac97_data(to_ac97_data));
				  
				  
		//Send signals to the P9 connector
	
	assign IO_P9[7] = ac97_bit_clock; // Pin 1
   assign IO_P9[6] = audio_reset_b;  // Pin 2
   assign IO_P9[5] = ac97_sdata_out; // Pin 3
   assign IO_P9[4]= ac97_sdata_in; // Pin 4
   assign IO_P9[3] = ac97_synch; // Pin 5
	assign IO_P9[2] = reset;   			// pin 6        
	assign IO_P9[1] = CLK_100MHz;			// pin 7

   // allow user to adjust volume
   wire vup,vdown;
   reg old_vup,old_vdown;
   debounce bup(.reset(reset),.clock(CLK_100MHz),.noisy(~vol_button_up),.clean(vup));
   debounce bdown(.reset(reset),.clock(CLK_100MHz),.noisy(~vol_button_down),.clean(vdown));
   reg [4:0] volume;
   always @ (posedge CLK_100MHz) begin
     if (reset) volume <= 5'd20;
     else begin
	if (vup & ~old_vup & volume != 5'd31) volume <= volume+5'd1;       
	if (vdown & ~old_vdown & volume != 5'd0) volume <= volume-5'd1;       
     end
     old_vup <= vup;
     old_vdown <= vdown;
   end
	
	
	
	// allow user to change the filter number
   wire fup,fdown;
   reg old_fup,old_fdown;
   debounce filtup(.reset(reset),.clock(CLK_100MHz),.noisy(~filt_up),.clean(fup));
   debounce filtdown(.reset(reset),.clock(CLK_100MHz),.noisy(~filt_down),.clean(fdown));
   reg [3:0] filtnum;
   always @ (posedge CLK_100MHz) begin
     if (reset) filtnum <= 4'd0;
     else begin
	if (fup & ~old_fup & filtnum != 4'd15) filtnum <= filtnum+4'h1;       
	if (fdown & ~old_fdown & filtnum != 4'd0) filtnum <= filtnum-4'h1;       
     end
     old_fup <= fup;
     old_fdown <= fdown;
   end
	
	FilterDisplay fdisplay(.filtnum(filtnum),.SevenSegment(SevenSegment),.SevenSegmentEnable(SevenSegmentEnable));

//   allow user to select step (frequency) up/down using buttons 4 (up) and 3 (down)
wire sup,sdown;
reg old_sup,old_sdown; 

debounce sbup(.reset(reset),.clock(CLK_100MHz),.noisy(~step_button_up),.clean(sup));
debounce sbdown(.reset(reset),.clock(CLK_100MHz),.noisy(~step_button_down),.clean(sdown));
   
reg [15:0] step;
   always @ (posedge CLK_100MHz) begin
     
	if (reset) 
	step <= 16'h0400;
     
	else begin
		if (sup & ~old_sup & step != 16'hFFFF)  
		step <= step + 16'h0F;       
		if (sdown & ~old_sdown & step != 16'h0) 
		step <= step - 16'h0F; 		     
	end
     
	old_sup <= sup;
	old_sdown <= sdown;
   
	end
endmodule

///////////////////////////////////////////////////////////////////////////////
// 
// Created by Robert Sikora May 2017
// to be compatible with Numato Lab MimasV2
//
///////////////////////////////////////////////////////////////////////////////

module FilterDisplay(
  input wire [3:0] filtnum,        
  output wire [7:0] SevenSegment,
  output wire [2:0] SevenSegmentEnable
	);

reg [2:0]  enable;
reg [7:0]  bcd [0:15];  

initial begin

	enable  <= 3'b101;

	bcd[0]  = 8'b00000011;
	bcd[1]  = 8'b10011111;
	bcd[2]  = 8'b00100101;
	bcd[3]  = 8'b00001101;
	bcd[4]  = 8'b10011001;
	bcd[5]  = 8'b01001001;
	bcd[6]  = 8'b01000001;
	bcd[7]  = 8'b00011111;
	bcd[8]  = 8'b00000001;
	bcd[9]  = 8'b00011001;
	bcd[10] = 8'b00010001;
	bcd[11] = 8'b11000001;
	bcd[12] = 8'b01100011;
	bcd[13] = 8'b10000101;
	bcd[14] = 8'b01100001;
	bcd[15] = 8'b01110001;

end

assign SevenSegmentEnable = enable;
assign SevenSegment = bcd[filtnum];

	//always@(filtnum)
		//begin
			
			
		
		//end

endmodule






///////////////////////////////////////////////////////////////////////////////
//
// Filter Control Module
// 
// Created by Robert Sikora May 2017
// to be compatible with Numato Lab MimasV2
//
///////////////////////////////////////////////////////////////////////////////

module FilterControlModule(
  input wire clock,	           // 27mhz system clock
  input wire reset,                // 1 to reset to initial state
  input wire ready,                // 1 when AC97 data is available
  input wire [15:0]step,             // 1 for playback, 0 for record
  input wire [3:0] filtnum,        // 1 when using low-pass filter
  input wire [7:0] from_ac97_data, // 8-bit PCM data from mic
  output reg [7:0] to_ac97_data    // 8-bit PCM data to headphone
);  
   // test: playback 750hz tone, or loopback using incoming data
   wire [19:0] tone;
   tone750hz xxx(.clock(clock),.ready(ready),.pcm_data(tone));
	
	wire [17:0] filtered;
	fir31 fir(.clock(clock),.reset(reset),.ready(ready),.x(from_ac97_data),.y(filtered),.filtnum(filtnum));
	
	wire [19:0] tone_2;
	ToneVarModule tone02(.clock(clock),.ready(ready),.step(step),.pcm_data(tone_2));
	
	wire signed [15:0] mult;
   MultiplierModule mul(.clock(clock), .ready(ready), .x(from_ac97_data),.y(tone_2[19:12]),.prod (mult));

   wire signed [17:0] delayed;
	DelayModule del(.clock(clock),.reset(reset),.ready(ready),.x(from_ac97_data), .y(delayed), .filtnum(filtnum));


   always @ (posedge clock) begin
      if (ready) begin
	    
			case (filtnum[3:0])
				4'h0:  to_ac97_data <= from_ac97_data;
				4'h1:  to_ac97_data <= filtered[17:10]; // Low Pass #1
				4'h2:  to_ac97_data <= filtered[17:10]; // Band Pass #2
				4'h3:  to_ac97_data <= filtered[17:10]; // High Pass #3
				4'h4:  to_ac97_data <= tone[19:12];
				4'h5:  to_ac97_data <= tone_2[19:12];
				4'h6:  to_ac97_data <= delayed[17:10];   // echo with filtnum = 6
				4'h7:  to_ac97_data <= delayed[17:10];   // reverb single delay with filtnum = 7
				4'h8:  to_ac97_data <= delayed[17:10];   // reverb 4 delays with filtnum = 8
				4'h9:  to_ac97_data <= mult[15:8];				
				default: to_ac97_data <= from_ac97_data;
			endcase
	 
      end
   end
endmodule


////////////////////////////////////////////////
//      MultiplierModule()
//
//  Multiply the two 8-bit input signals x and y
//  The output is a 16 bit signal called prod 
// 
//  Created by Robert Sikora May 2017
//
///////////////////////////////////////////////////////////////////////////////

module MultiplierModule(
   input wire clock,
	input wire ready,
	input wire signed [7:0] x,
	input wire signed [7:0] y,
	output reg signed [15:0] prod
);

   always @(posedge clock) begin
	   if(ready) 
		   prod <= x * y  ;			
	end
endmodule	


///////////////////////////////////////////////////////////////////////////////
//
// 6.111 FPGA Labkit -- tone750hz module
//
// For Labkit Revision 004
// Created: October 31, 2004, from revision 003 file
// Author: Nathan Ickes, 6.111 staff
//
///////////////////////////////////////////////////////////////////////////////

module tone750hz (
  input wire clock,
  input wire ready,
  output reg [19:0] pcm_data
);
   reg [8:0] index;

   initial begin
      index <= 8'h00;
      // synthesis attribute init of index is "00";
      pcm_data <= 20'h00000;
      // synthesis attribute init of pcm_data is "00000";
   end
   
   always @(posedge clock) begin
      if (ready) index <= index+9'h1;
   end
   
   // one cycle of a sinewave in 64 20-bit samples
   always @(index) begin
      case (index[5:0])
        6'h00: pcm_data <= 20'h00000;
        6'h01: pcm_data <= 20'h0C8BD;
        6'h02: pcm_data <= 20'h18F8B;
        6'h03: pcm_data <= 20'h25280;
        6'h04: pcm_data <= 20'h30FBC;
        6'h05: pcm_data <= 20'h3C56B;
        6'h06: pcm_data <= 20'h471CE;
        6'h07: pcm_data <= 20'h5133C;
        6'h08: pcm_data <= 20'h5A827;
        6'h09: pcm_data <= 20'h62F20;
        6'h0A: pcm_data <= 20'h6A6D9;
        6'h0B: pcm_data <= 20'h70E2C;
        6'h0C: pcm_data <= 20'h7641A;
        6'h0D: pcm_data <= 20'h7A7D0;
        6'h0E: pcm_data <= 20'h7D8A5;
        6'h0F: pcm_data <= 20'h7F623;
        6'h10: pcm_data <= 20'h7FFFF;
        6'h11: pcm_data <= 20'h7F623;
        6'h12: pcm_data <= 20'h7D8A5;
        6'h13: pcm_data <= 20'h7A7D0;
        6'h14: pcm_data <= 20'h7641A;
        6'h15: pcm_data <= 20'h70E2C;
        6'h16: pcm_data <= 20'h6A6D9;
        6'h17: pcm_data <= 20'h62F20;
        6'h18: pcm_data <= 20'h5A827;
        6'h19: pcm_data <= 20'h5133C;
        6'h1A: pcm_data <= 20'h471CE;
        6'h1B: pcm_data <= 20'h3C56B;
        6'h1C: pcm_data <= 20'h30FBC;
        6'h1D: pcm_data <= 20'h25280;
        6'h1E: pcm_data <= 20'h18F8B;
        6'h1F: pcm_data <= 20'h0C8BD;
        6'h20: pcm_data <= 20'h00000;
        6'h21: pcm_data <= 20'hF3743;
        6'h22: pcm_data <= 20'hE7075;
        6'h23: pcm_data <= 20'hDAD80;
        6'h24: pcm_data <= 20'hCF044;
        6'h25: pcm_data <= 20'hC3A95;
        6'h26: pcm_data <= 20'hB8E32;
        6'h27: pcm_data <= 20'hAECC4;
        6'h28: pcm_data <= 20'hA57D9;
        6'h29: pcm_data <= 20'h9D0E0;
        6'h2A: pcm_data <= 20'h95927;
        6'h2B: pcm_data <= 20'h8F1D4;
        6'h2C: pcm_data <= 20'h89BE6;
        6'h2D: pcm_data <= 20'h85830;
        6'h2E: pcm_data <= 20'h8275B;
        6'h2F: pcm_data <= 20'h809DD;
        6'h30: pcm_data <= 20'h80000;
        6'h31: pcm_data <= 20'h809DD;
        6'h32: pcm_data <= 20'h8275B;
        6'h33: pcm_data <= 20'h85830;
        6'h34: pcm_data <= 20'h89BE6;
        6'h35: pcm_data <= 20'h8F1D4;
        6'h36: pcm_data <= 20'h95927;
        6'h37: pcm_data <= 20'h9D0E0;
        6'h38: pcm_data <= 20'hA57D9;
        6'h39: pcm_data <= 20'hAECC4;
        6'h3A: pcm_data <= 20'hB8E32;
        6'h3B: pcm_data <= 20'hC3A95;
        6'h3C: pcm_data <= 20'hCF044;
        6'h3D: pcm_data <= 20'hDAD80;
        6'h3E: pcm_data <= 20'hE7075;
        6'h3F: pcm_data <= 20'hF3743;
      endcase // case(index[5:0])
   end // always @ (index)
endmodule

///////////////////////////////////////////////////////////////////////////////
//
// ToneVarModule()
//
// based on tone750hz module
// For Labkit Revision 004
// Created: October 31, 2004, from revision 003 file
// Author: Nathan Ickes, 6.111 staff
// 
// Modified by Robert Sikora May 2017
//
// generate PCM data for a variable frequency sine wave (assuming f(ready) = 48khz)
// Look up table of 64 values of 20 bits.
// The input value of step (16 bits) determines the frequency.
// Only the upper 6 bits of step are used giving 64 possible values for the location
// in the look up table.
// So a value of step = 1024 would require 64 increments of index for a full cycle.
// The output frequency would be 48 kHz/64 = 750 Hz.
// If step is 1, the frequency would be 1024 times slower or about 0.7 Hz 
///////////////////////////////////////////////////////////////////////////////
module ToneVarModule (
  input wire clock,
  input wire ready,
  input wire [15:0] step,
  output reg [19:0] pcm_data
);
   reg [15:0] index;

   initial begin
      index <= 8'h00;
      // synthesis attribute init of index is "00";
      pcm_data <= 20'h00000;
      // synthesis attribute init of pcm_data is "00000";
   end
   
   always @(posedge clock) begin
      if (ready) index <= index + step;
   end
   
   // one cycle of a sinewave in 64 20-bit samples
   always @(index) begin
      case (index[15:10])
        6'h00: pcm_data <= 20'h00000;
        6'h01: pcm_data <= 20'h0C8BD;
        6'h02: pcm_data <= 20'h18F8B;
        6'h03: pcm_data <= 20'h25280;
        6'h04: pcm_data <= 20'h30FBC;
        6'h05: pcm_data <= 20'h3C56B;
        6'h06: pcm_data <= 20'h471CE;
        6'h07: pcm_data <= 20'h5133C;
        6'h08: pcm_data <= 20'h5A827;
        6'h09: pcm_data <= 20'h62F20;
        6'h0A: pcm_data <= 20'h6A6D9;
        6'h0B: pcm_data <= 20'h70E2C;
        6'h0C: pcm_data <= 20'h7641A;
        6'h0D: pcm_data <= 20'h7A7D0;
        6'h0E: pcm_data <= 20'h7D8A5;
        6'h0F: pcm_data <= 20'h7F623;
        6'h10: pcm_data <= 20'h7FFFF;
        6'h11: pcm_data <= 20'h7F623;
        6'h12: pcm_data <= 20'h7D8A5;
        6'h13: pcm_data <= 20'h7A7D0;
        6'h14: pcm_data <= 20'h7641A;
        6'h15: pcm_data <= 20'h70E2C;
        6'h16: pcm_data <= 20'h6A6D9;
        6'h17: pcm_data <= 20'h62F20;
        6'h18: pcm_data <= 20'h5A827;
        6'h19: pcm_data <= 20'h5133C;
        6'h1A: pcm_data <= 20'h471CE;
        6'h1B: pcm_data <= 20'h3C56B;
        6'h1C: pcm_data <= 20'h30FBC;
        6'h1D: pcm_data <= 20'h25280;
        6'h1E: pcm_data <= 20'h18F8B;
        6'h1F: pcm_data <= 20'h0C8BD;
        6'h20: pcm_data <= 20'h00000;
        6'h21: pcm_data <= 20'hF3743;
        6'h22: pcm_data <= 20'hE7075;
        6'h23: pcm_data <= 20'hDAD80;
        6'h24: pcm_data <= 20'hCF044;
        6'h25: pcm_data <= 20'hC3A95;
        6'h26: pcm_data <= 20'hB8E32;
        6'h27: pcm_data <= 20'hAECC4;
        6'h28: pcm_data <= 20'hA57D9;
        6'h29: pcm_data <= 20'h9D0E0;
        6'h2A: pcm_data <= 20'h95927;
        6'h2B: pcm_data <= 20'h8F1D4;
        6'h2C: pcm_data <= 20'h89BE6;
        6'h2D: pcm_data <= 20'h85830;
        6'h2E: pcm_data <= 20'h8275B;
        6'h2F: pcm_data <= 20'h809DD;
        6'h30: pcm_data <= 20'h80000;
        6'h31: pcm_data <= 20'h809DD;
        6'h32: pcm_data <= 20'h8275B;
        6'h33: pcm_data <= 20'h85830;
        6'h34: pcm_data <= 20'h89BE6;
        6'h35: pcm_data <= 20'h8F1D4;
        6'h36: pcm_data <= 20'h95927;
        6'h37: pcm_data <= 20'h9D0E0;
        6'h38: pcm_data <= 20'hA57D9;
        6'h39: pcm_data <= 20'hAECC4;
        6'h3A: pcm_data <= 20'hB8E32;
        6'h3B: pcm_data <= 20'hC3A95;
        6'h3C: pcm_data <= 20'hCF044;
        6'h3D: pcm_data <= 20'hDAD80;
        6'h3E: pcm_data <= 20'hE7075;
        6'h3F: pcm_data <= 20'hF3743;
      endcase // case(index[5:0])
   end // always @ (index)
endmodule




///////////////////////////////////////////////////////////////////////////////
//
// 6.111 FPGA Labkit -- mybram module
//
// For Labkit Revision 004
// Created: October 31, 2004, from revision 003 file
// Author: Nathan Ickes, 6.111 staff
//
// Verilog equivalent to a BRAM, tools will infer the right thing!
// number of locations = 1<<LOGSIZE, width in bits = WIDTH.
// default is a 16K x 1 memory.
//
///////////////////////////////////////////////////////////////////////////////

module mybram #(parameter LOGSIZE=14, WIDTH=1)
              (input wire [LOGSIZE-1:0] addr,
               input wire clk,
               input wire [WIDTH-1:0] din,
               output reg [WIDTH-1:0] dout,
               input wire we);
   // let the tools infer the right number of BRAMs
   (* ram_style = "block" *)
   reg [WIDTH-1:0] mem[(1<<LOGSIZE)-1:0];
   always @(posedge clk) begin
     if (we) mem[addr] <= din;
     dout <= mem[addr];
   end
endmodule



///////////////////////////////////////////////////////////////////////////////
//
// 6.111 FPGA Labkit -- coeffs31 module
//
// For Labkit Revision 004
// Created: October 31, 2004, from revision 003 file
// Author: Nathan Ickes, 6.111 staff
// 
// Modified by Robert Sikora May 2017
// to get multiple filters using the input select
//
// Coefficients for a 31-tap low-pass FIR filter with Wn=.125 (eg, 3kHz for a
// 48kHz sample rate).  Since we're doing integer arithmetic, we've scaled
// the coefficients by 2**10
// Matlab command: round(fir1(30,.125)*1024)
//
///////////////////////////////////////////////////////////////////////////////

module coeffs31(
  input wire [3:0] select,
  input wire [4:0] index,
  output reg signed [9:0] coeff
);
  // tools will turn this into a 31x10 ROM
  always @(index or select)
     case (select[3:0])           //Which filter   
4'd1: begin //select Filter 1 LOWPASS 3 kHz
         case (index[4:0]) 
			  5'd0:  coeff = -10'sd1;  // -10sd1;
           5'd1:  coeff = -10'sd2;  // -10sd1;
           5'd2:  coeff = -10'sd3;   //  -10sd3;
           5'd3:  coeff = -10'sd5;   //  -10sd5;
           5'd4:  coeff = -10'sd6;   //  -10sd6;
           5'd5:  coeff = -10'sd7;   //  -10sd7;
           5'd6:  coeff = -10'sd5;   //  -10sd5;
           5'd7:  coeff = -10'sd0;    //  -10sd0;
           5'd8:  coeff = 10'sd11;
           5'd9:  coeff = 10'sd27;
           5'd10: coeff = 10'sd47;
           5'd11: coeff = 10'sd69;
           5'd12: coeff = 10'sd91;
           5'd13: coeff = 10'sd110;  // 110  // I needed to reduce these values to avoid
           5'd14: coeff = 10'sd122;  // 123  // over-range distortion.
           5'd15: coeff = 10'sd127;  // 128
           5'd16: coeff = 10'sd122;  // 123
           5'd17: coeff = 10'sd110;  // 110
           5'd18: coeff = 10'sd91;
           5'd19: coeff = 10'sd69;
           5'd20: coeff = 10'sd47;
           5'd21: coeff = 10'sd27;
           5'd22: coeff = 10'sd11;
           5'd23: coeff = 10'sd0;
           5'd24: coeff = -10'sd5;
           5'd25: coeff = -10'sd7;
           5'd26: coeff = -10'sd6;
           5'd27: coeff = -10'sd5;
           5'd28: coeff = -10'sd3;
           5'd29: coeff = -10'sd2;
           5'd30: coeff = -10'sd1;
           default: coeff = 10'h0;
         endcase // End Filter 1
end 
4'd2: begin  // select Filter 2 BANDPASS 12kHz
         case (index[4:0])          // Which coefficient of filter 2
           5'd0:  coeff = -10'sd8;  // -10sd1;
           5'd1:  coeff = -10'sd9;  // -10sd1;
           5'd2:  coeff = -10'sd13;   //  -10sd3;
           5'd3:  coeff = -10'sd18;   //  -10sd5;
           5'd4:  coeff = -10'sd22;   //  -10sd6;
           5'd5:  coeff = -10'sd25;   //  -10sd7;
           5'd6:  coeff = -10'sd24;   //  -10sd5;
           5'd7:  coeff = -10'sd17;    //  -10sd0;
           5'd8: coeff =  -10'sd4;
           5'd9: coeff =   10'sd15;
           5'd10: coeff =  10'sd39;
           5'd11: coeff =  10'sd65;
           5'd12: coeff =  10'sd90;
           5'd13: coeff =  10'sd112;
           5'd14: coeff =  10'sd126;
           5'd15:  coeff = 10'sd131;  //!
           5'd16:  coeff = 10'sd126;
           5'd17: coeff =  10'sd112;
           5'd18: coeff =  10'sd90;
           5'd19: coeff =  10'sd65;
           5'd20: coeff =  10'sd39;  // 110  // I needed to reduce these values to avoid
           5'd21: coeff =  10'sd15;  // 123  // over-range distortion.
           5'd22: coeff = -10'sd4;  // 128
           5'd23: coeff = -10'sd17;  // 123
           5'd24: coeff = -10'sd24;  // 110
           5'd25: coeff = -10'sd25;
           5'd26: coeff = -10'sd22;
           5'd27: coeff = -10'sd18;
           5'd28: coeff = -10'sd13;
           5'd29: coeff = -10'sd9;
           5'd30: coeff = -10'sd8;
           default: coeff = 10'h0;
         endcase  // End Filter 2
		 end
		 
		 4'd3: begin  // select Filter 3 HIGHPASS 12kHz
         case (index[4:0])          // Which coefficient of filter 3
           5'd0:  coeff =  10'sd1;  // -10sd1;
           5'd1:  coeff =  10'sd1;  // -10sd1;
           5'd2:  coeff =  10'sd1;   //  -10sd3;
           5'd3:  coeff =  10'sd1;   //  -10sd5;
           5'd4:  coeff =  10'sd1;   //  -10sd6;
           5'd5:  coeff =  10'sd0;   //  -10sd7;
           5'd6:  coeff = -10'sd2;   //  -10sd5;
           5'd7:  coeff = -10'sd6;    //  -10sd0;
           5'd8: coeff =  -10'sd11;
           5'd9: coeff =  -10'sd18;
           5'd10: coeff = -10'sd25;
           5'd11: coeff = -10'sd33;
           5'd12: coeff = -10'sd40;
           5'd13: coeff = -10'sd46;
           5'd14: coeff = -10'sd49;
           5'd15:  coeff = 10'sd462;  //!
           5'd16:  coeff =-10'sd49;
           5'd17: coeff = -10'sd46;
           5'd18: coeff = -10'sd40;
           5'd19: coeff = -10'sd33;
           5'd20: coeff = -10'sd25;  // 110  // I needed to reduce these values to avoid
           5'd21: coeff = -10'sd18;  // 123  // over-range distortion.
           5'd22: coeff = -10'sd11;  // 128
           5'd23: coeff = -10'sd6;  // 123
           5'd24: coeff = -10'sd2;  // 110
           5'd25: coeff = -10'sd0;
           5'd26: coeff =  10'sd1;
           5'd27: coeff =  10'sd1;
           5'd28: coeff =  10'sd1;
           5'd29: coeff =  10'sd1;
           5'd30: coeff =  10'sd1;
           default: coeff = 10'h0;
         endcase  // End Filter 3
		 end
		 
		 default: coeff = 10'h0;  // didn't find selected filter ( should be 1, 2 OR 3)
     endcase

endmodule


///////////////////////////////////////////////////////////////////////////////
// 
// Created by Robert Sikora May 2017
// Return the delay that is used by DelayModule
//
///////////////////////////////////////////////////////////////////////////////

module delay4k(
  input wire [3:0] index,
  
output reg [11:0] delay
);
  // tools will turn this into a 16 x 12bit ROM
  

always @(index)
 
     case (index[3:0])          
           4'd0:  delay =  12'd4079; 
           4'd1:  delay =  12'd3469;  
           4'd2:  delay =  12'd2579;   
           4'd3:  delay =  12'd1823;   
           4'd4:  delay =  12'd1;   
           4'd5:  delay =  12'd1;   
           4'd6:  delay =  12'd1;   
           4'd7:  delay =  12'd1;   
           4'd8:  delay =  12'd1;
           4'd9:  delay =  12'd1;
           4'd10: delay =  12'd1;
           4'd11: delay =  12'd1;
           4'd12: delay =  12'd1;
           4'd13: delay =  12'd1;  
           4'd14: delay =  12'd1;  
           4'd15: delay =  12'd1;    
           default: delay = 12'd1;
      
		endcase 
endmodule


///////////////////////////////////////////////////////////////////////////////
//
// Debounce Module
//
// For Labkit Revision 004
// Created: October 31, 2004, from revision 003 file
// Author: Nathan Ickes, 6.111 staff
//
///////////////////////////////////////////////////////////////////////////////


module debounce (
  input wire reset, clock, noisy,
  output reg clean
);
  reg [18:0] count;
  reg new;

  always @(posedge clock)
    if (reset) begin
      count <= 0;
      new <= noisy;
      clean <= noisy;
    end
    else if (noisy != new) begin
      // noisy input changed, restart the .01 sec clock
      new <= noisy;
      count <= 0;
    end
    else if (count == 270000)
      // noisy input stable for .01 secs, pass it along!
      clean <= new;
    else
      // waiting for .01 sec to pass
      count <= count+1'b1;

endmodule


///////////////////////////////////////////////////////////////////////////////
//
// 6.111 FPGA Labkit -- fir31 module
//
// For Labkit Revision 004
// Created: October 31, 2004, from revision 003 file
// Author: Nathan Ickes, 6.111 staff
// 
// Modified by Robert Sikora May 2017
// to make a functioning filter.
//
// 31-tap FIR filter, 8-bit signed data, 10-bit signed coefficients.
// ready is asserted whenever there is a new sample on the X input,
// the Y output should also be sampled at the same time.  Assumes at
// least 32 clocks between ready assertions.  Note that since the
// coefficients have been scaled by 2**10, so has the output (it's
// expanded from 8 bits to 18 bits).  To get an 8-bit result from the
// filter just divide by 2**10, ie, use Y[17:10].
//
///////////////////////////////////////////////////////////////////////////////

module fir31(
  input wire clock,reset,ready,
  input wire signed [7:0] x,
  output wire signed [17:0] y,
  input wire [3:0] filtnum
);

    reg [4:0] index;
	 reg [4:0] offset;
    reg signed [7:0] sample [0:31];
	 reg signed [17:0] accum;
	 reg signed [17:0] output_reg;
	 wire signed [9:0] coeff;
	 reg done;

		assign y = output_reg;
   
    initial begin

        index = 5'h0;
			accum <= 18'd0;
		  output_reg <= 5'h0;
       
		 
		sample[0] <= 0;
	  sample[1] <= 0;
	  sample[2] <= 0;
	  sample[3] <= 0;
	  sample[4] <= 0;
	  sample[5] <= 0;
	  sample[6] <= 0;
	  sample[7] <= 0;
	  sample[8] <= 0;
	  sample[9] <= 0;
	  sample[10] <= 0;
	  sample[11] <= 0;
	  sample[12] <= 0;
	  sample[13] <= 0;
	  sample[14] <= 0;
	  sample[15] <= 0;
	  sample[16] <= 0;
	  sample[17] <= 0;
	  sample[18] <= 0;
	  sample[19] <= 0;
	  sample[20] <= 0;
	  sample[21] <= 0;
	  sample[22] <= 0;
	  sample[23] <= 0;
	  sample[24] <= 0;
	  sample[25] <= 0;
	  sample[26] <= 0;
	  sample[27] <= 0;
	  sample[28] <= 0;
	  sample[29] <= 0;
	  sample[30] <= 0;
	  sample[31] <= 0;

    end
   
  always @(posedge clock) begin   // could this increment more than once on each "ready" ?
     if (ready) begin
         offset <= offset + 5'h1;
     end			
  
end

  coeffs31 coe(filtnum, index,coeff);

  always @(posedge clock) begin
    if(ready) begin
	    sample[offset] <= x;	
		 
index  <= 0;		
       accum  <= 0;
       done   <= 0;		 
	 end
	 
    if (!ready && (index < 30) ) begin   // calculate when ready is low. (< 30)
	    accum <= accum + coeff * sample[offset-index];
	    index <= index + 5'h1;
    end		
    if (!done && (index == 30) ) begin   // when finished, update output (== 30)
		 output_reg <= accum;   // update ouput register, once.
		 done <= 1'b1;           
    end	

  end
  
endmodule	




///////////////////////////////////////////////////////////////////////////////
//
// 6.111 FPGA Labkit -- lab4audio module
//
// For Labkit Revision 004
// Created: October 31, 2004, from revision 003 file
// Author: Nathan Ickes, 6.111 staff
// 
// Modified by Robert Sikora May 2017
// to be compatible with Numato Lab MimasV2
//
// bi-directional monaural interface to AC97
//
///////////////////////////////////////////////////////////////////////////////

module lab4audio (
  input wire CLK_100MHz,
  input wire reset,
  input wire [4:0] volume,
  output wire [7:0] audio_in_data,
  input wire [7:0] audio_out_data,
  output wire ready,
  output reg audio_reset_b,   // ac97 interface signals
  output wire ac97_sdata_out,
  input wire ac97_sdata_in,
  output wire ac97_synch,
  input wire ac97_bit_clock
);

  wire [7:0] command_address;
  wire [15:0] command_data;
  wire command_valid;
  wire [19:0] left_in_data, right_in_data;
  wire [19:0] left_out_data, right_out_data;

  // wait a little before enabling the AC97 codec
  reg [9:0] reset_count;
  always @(posedge CLK_100MHz) begin
    if (reset) begin
      audio_reset_b = 1'b0;
      reset_count = 0;
    end else if (reset_count == 1023)
      audio_reset_b = 1'b1;
    else
      reset_count = reset_count+ 10'h1;
  end

  wire ac97_ready;
  ac97 ac97(.ready(ac97_ready),
            .command_address(command_address),
            .command_data(command_data),
            .command_valid(command_valid),
            .left_data(left_out_data), .left_valid(1'b1),
            .right_data(right_out_data), .right_valid(1'b1),
            .left_in_data(left_in_data), .right_in_data(right_in_data),
            .ac97_sdata_out(ac97_sdata_out),
            .ac97_sdata_in(ac97_sdata_in),
            .ac97_synch(ac97_synch),
            .ac97_bit_clock(ac97_bit_clock));

  // ready: one cycle pulse synchronous with CLK_100MHz
  reg [2:0] ready_sync;
  always @ (posedge CLK_100MHz) ready_sync <= {ready_sync[1:0], ac97_ready};
  assign ready = ready_sync[1] & ~ready_sync[2];

  reg [7:0] out_data;
  always @ (posedge CLK_100MHz)
    if (ready) out_data <= audio_out_data;
  assign audio_in_data = left_in_data[19:12];
  assign left_out_data = {out_data, 12'b000000000000};
  assign right_out_data = left_out_data;

  // generate repeating sequence of read/writes to AC97 registers
  ac97commands cmds(.clock(CLK_100MHz), .ready(ready),
                    .command_address(command_address),
                    .command_data(command_data),
                    .command_valid(command_valid),
                    .volume(volume),
                    .source(3'b000));     // mic
endmodule



///////////////////////////////////////////////////////////////////////////////
//
// 6.111 FPGA Labkit -- ac97 module
//
// For Labkit Revision 004
// Created: October 31, 2004, from revision 003 file
// Author: Nathan Ickes, 6.111 staff
// 
// Modified by Robert Sikora May 2017
// to be compatible with Numato Lab MimasV2
//
///////////////////////////////////////////////////////////////////////////////

// assemble/disassemble AC97 serial frames
module ac97 (
  output reg ready,
  input wire [7:0] command_address,
  input wire [15:0] command_data,
  input wire command_valid,
  input wire [19:0] left_data,
  input wire left_valid,
  input wire [19:0] right_data,
  input wire right_valid,
  output reg [19:0] left_in_data, right_in_data,
  output reg ac97_sdata_out,
  input wire ac97_sdata_in,
  output reg ac97_synch,
  input wire ac97_bit_clock
);
  reg [7:0] bit_count;

  reg [19:0] l_cmd_addr;
  reg [19:0] l_cmd_data;
  reg [19:0] l_left_data, l_right_data;
  reg l_cmd_v, l_left_v, l_right_v;

  initial begin
    ready <= 1'b0;
    // synthesis attribute init of ready is "0";
    ac97_sdata_out <= 1'b0;
    // synthesis attribute init of ac97_sdata_out is "0";
    ac97_synch <= 1'b0;
    // synthesis attribute init of ac97_synch is "0";

    bit_count <= 8'h00;
    // synthesis attribute init of bit_count is "0000";
    l_cmd_v <= 1'b0;
    // synthesis attribute init of l_cmd_v is "0";
    l_left_v <= 1'b0;
    // synthesis attribute init of l_left_v is "0";
    l_right_v <= 1'b0;
    // synthesis attribute init of l_right_v is "0";

    left_in_data <= 20'h00000;
    // synthesis attribute init of left_in_data is "00000";
    right_in_data <= 20'h00000;
    // synthesis attribute init of right_in_data is "00000";
  end

  always @(posedge ac97_bit_clock) begin
    // Generate the sync signal
    if (bit_count == 255)
      ac97_synch <= 1'b1;
    if (bit_count == 15)
      ac97_synch <= 1'b0;

    // Generate the ready signal
    if (bit_count == 128)
      ready <= 1'b1;
    if (bit_count == 2)
      ready <= 1'b0;

    // Latch user data at the end of each frame. This ensures that the
    // first frame after reset will be empty.
    if (bit_count == 255) begin
      l_cmd_addr <= {command_address, 12'h000};
      l_cmd_data <= {command_data, 4'h0};
      l_cmd_v <= command_valid;
      l_left_data <= left_data;
      l_left_v <= left_valid;
      l_right_data <= right_data;
      l_right_v <= right_valid;
    end

    if ((bit_count >= 0) && (bit_count <= 15))
      // Slot 0: Tags
      case (bit_count[3:0])
        4'h0: ac97_sdata_out <= 1'b1;      // Frame valid
        4'h1: ac97_sdata_out <= l_cmd_v;   // Command address valid
        4'h2: ac97_sdata_out <= l_cmd_v;   // Command data valid
        4'h3: ac97_sdata_out <= l_left_v;  // Left data valid
        4'h4: ac97_sdata_out <= l_right_v; // Right data valid
        default: ac97_sdata_out <= 1'b0;
      endcase
    else if ((bit_count >= 16) && (bit_count <= 35))
      // Slot 1: Command address (8-bits, left justified)
      ac97_sdata_out <= l_cmd_v ? l_cmd_addr[35-bit_count] : 1'b0;
    else if ((bit_count >= 36) && (bit_count <= 55))
      // Slot 2: Command data (16-bits, left justified)
      ac97_sdata_out <= l_cmd_v ? l_cmd_data[55-bit_count] : 1'b0;
    else if ((bit_count >= 56) && (bit_count <= 75)) begin
      // Slot 3: Left channel
      ac97_sdata_out <= l_left_v ? l_left_data[19] : 1'b0;
      l_left_data <= { l_left_data[18:0], l_left_data[19] };
    end
    else if ((bit_count >= 76) && (bit_count <= 95))
      // Slot 4: Right channel
      ac97_sdata_out <= l_right_v ? l_right_data[95-bit_count] : 1'b0;
    else
      ac97_sdata_out <= 1'b0;

    bit_count <= bit_count+1;
  end // always @ (posedge ac97_bit_clock)

  always @(negedge ac97_bit_clock) begin
    if ((bit_count >= 57) && (bit_count <= 76))
      // Slot 3: Left channel
      left_in_data <= { left_in_data[18:0], ac97_sdata_in };
    else if ((bit_count >= 77) && (bit_count <= 96))
      // Slot 4: Right channel
      right_in_data <= { right_in_data[18:0], ac97_sdata_in };
  end
endmodule


///////////////////////////////////////////////////////////////////////////////
//
// 6.111 FPGA Labkit -- ac97commands module
//
// For Labkit Revision 004
// Created: October 31, 2004, from revision 003 file
// Author: Nathan Ickes, 6.111 staff
// 
// Modified by Robert Sikora May 2017
// to be compatible with Numato Lab MimasV2
//
///////////////////////////////////////////////////////////////////////////////
// issue initialization commands to AC97
module ac97commands (
  input wire clock,
  input wire ready,
  output wire [7:0] command_address,
  output wire [15:0] command_data,
  output reg command_valid,
  input wire [4:0] volume,
  input wire [2:0] source
);
  reg [23:0] command;

  reg [3:0] state;
  initial begin
    command <= 4'h0;
    // synthesis attribute init of command is "0";
    command_valid <= 1'b0;
    // synthesis attribute init of command_valid is "0";
    state <= 16'h0000;
    // synthesis attribute init of state is "0000";
  end

  assign command_address = command[23:16];
  assign command_data = command[15:0];

  wire [4:0] vol;
  assign vol = 31-volume;  // convert to attenuation

  always @(posedge clock) begin
    if (ready) state <= state+ 4'h1;

    case (state)
      4'h0: // Read ID
        begin
          command <= 24'h80_0000;
          command_valid <= 1'b1;
        end
      4'h1: // Read ID
        command <= 24'h80_0000;
      4'h3: // headphone volume
        command <= { 8'h04, 3'b000, vol, 3'b000, vol };
      4'h5: // PCM volume
        command <= 24'h18_0808;
      4'h6: // Record source select
        command <= { 8'h1A, 5'b00000, source, 5'b00000, source};
      4'h7: // Record gain = max
        command <= 24'h1C_0F0F;
      4'h9: // set +20db mic gain
        command <= 24'h0E_8048;
      4'hA: // Set beep volume
        command <= 24'h0A_0000;
      4'hB: // PCM out bypass mix1
        command <= 24'h20_8000;
      default:
        command <= 24'h80_0000;
    endcase // case(state)
  end // always @ (posedge clock)
endmodule // ac97commands

///////////////////////////////////////////////////////////////////////////////
//
// generate PCM data for 750hz sine wave (assuming f(ready) = 48khz)
//
///////////////////////////////////////////////////////////////////////////////














//////////////////////////////////////////////////////////////////////////////
//                         DelayModule()
///////////////////////////////////////////////////////////////////////////////
// 
// Created by Robert Sikora May 2017
// 
// Stores audio input samples in an 8 bit x 4k long array and will output the 
// sum of the set of samples[m] returned by an instance of the module delay4k.
// For example, this could be the most recent sample plus the sample from
// 4000 samples ago. This would give an echo from 1/48kHz * 4000 seconds ago. 
// Or about 80 ms.
///////////////////////////////////////////////////////////////////////////////


module DelayModule(
input wire clock,reset,ready, 
input wire signed [7:0] x,
output wire signed [17:0] y,
input wire [3:0] filtnum
);
 
reg [11:0]  offset;   // offset to location of current audio data in circular buffer. 
reg [3:0]  index;    // index to case statement
reg         done;     // 

  reg signed [7:0]  sample[0:4096];
  reg signed [17:0] accum;
  reg signed [17:0] output_reg;
  wire       [11:0] delay; 
  reg        [7:0]  size;
  reg signed [9:0]  revamp;
  
 
   assign y    = output_reg;   // output_reg;

//	assign y = {x, 10'h00};     // for straight through
   

initial begin
offset    <= 12'h0;
     
index     <= 4'h0;
     
accum     <= 18'd0;
output_reg <= 18'd0;	  	  
sample[0] <= 0;

end

 

// This module demonstrates both a single echo (filtnum = 6)
// and reverberation (filtnum = 7).
// For echo, use only the first value of delay (size = 1)and increase the 
// amplitude of that echo so that it is very distinct (revamp = 1FF).
// For reverberation, four delays are used. Since the samples from
// these different delays are all added, the scale factor needs
// to be smaller, otherwise the amplitude could go beyond a valid range. (revamp = 0AF)
//

  always @(posedge clock) begin   // could this increment more than once on each "ready" ?
     if (ready) begin
        offset <= offset + 12'h1;  //  Increment the offset once on each ready (when new audio sample) 
        if (filtnum == 6) begin    // Ech0
           size   <= 8'd1;
           revamp <= 10'sh1FF;
        end		 

		  else if (filtnum == 7) begin  // Reverberation
		     size   <= 8'd1;
			  revamp <= 10'sh1FF;
		  
		  end
 
		  else begin
		     size <= 8'd4;
			  revamp <= 10'sh0DF;
		  end

     end
end



  delay4k del(index,delay);

  always @(posedge clock) begin
    if(ready) begin	
		 
		 index  <= 0;		
       accum  <= 10'sh1FF * x;   // 1FF put the current sample into the accumulator
       done   <= 0;		 
	 end

    if (!ready && (index < size) ) begin   // calculate when ready is low. (< 2)
	    accum <= accum + revamp * sample[offset - delay];
	    index <= index + 4'd1;
    end		
    if (!done && (index == size) ) begin         // when finished, update output (== 2)

       if(filtnum == 6 ) begin
				sample[offset] <=   x;               // for echo                 		 
       end
		 
       else begin	    
			   sample[offset] <=   accum[17:10];     // for reverberation
	    end

       output_reg     <=  accum;
       done <= 1;           
    end	

  end
  
endmodule















