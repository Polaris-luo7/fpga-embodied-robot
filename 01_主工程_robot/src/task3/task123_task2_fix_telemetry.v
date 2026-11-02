// Read-only UART: 115200 8N1, snapshot every 200ms, no control receive path.
module task123_task2_fix_telemetry #(
 parameter integer CLK_HZ=50000000,BAUD=115200,FRAME_CYCLES=CLK_HZ/5,SENSOR_COUNT=3,ENCODER_REPORT=0,ERROR_REPORT=0
)(input wire clk,rst_n,
 input wire [15:0] front_mm,left_mm,right_mm,
 input wire [2:0] sensor_fault,motor_fault_code,
 input wire [1:0] speed_level,
 input wire [3:0] route_state,
 input wire armed,run_request,drive_enabled,emergency_stop,fault,
 input wire [127:0] positions,input wire [63:0] velocities,
 input wire [31:0] encoder_errors,input wire [3:0] fault_wheels,
 input wire imu_valid,input wire [31:0] roll_q16,pitch_q16,yaw_q16,
 input wire [66:0] route_diagnostic,
 input wire [2:0] sample_error,fault_sample_error,
 input wire [5:0] history_index,input wire [6:0] history_count,
 input wire history_frozen,input wire [199:0] history_word,
 output wire frame_done,output wire tx);
 localparam [2:0] IDLE=0,CONVERT=1,ISSUE=2,WAIT_TX=3;
 localparam integer BASE_BYTES=ENCODER_REPORT ? 159 : 79;
 localparam integer OLD_BYTES=BASE_BYTES+(ERROR_REPORT ? 29 : 0);
 localparam integer NORMAL_BYTES=OLD_BYTES+41;
 localparam integer ROUTE_BYTES=NORMAL_BYTES+46;
 localparam integer FRAME_BYTES=ROUTE_BYTES+81;
 localparam [7:0] ERROR_OFFSET=BASE_BYTES-2;
 reg [2:0] phase;
 reg [31:0] period_count;
 reg [47:0] values;
 reg [59:0] digits;
 reg [19:0] bcd;
 reg [15:0] binary;
 reg [4:0] convert_bit;
 reg [1:0] convert_channel;
 reg [9:0] flags;
 reg [3:0] state_q;
 reg [2:0] motor_q;
 reg [8:0] byte_index;
 reg [66:0] diagnostic_q;
 reg [2:0] sample_error_q,fault_sample_error_q;
 reg [5:0] history_index_q;reg [6:0] history_count_q;
 reg history_frozen_q;reg [199:0] history_word_q;
 reg [127:0] positions_q; reg [63:0] velocities_q;
 reg [31:0] errors_q;reg [3:0] fault_wheels_q;
 reg imu_q;reg [31:0] roll_q,pitch_q,yaw_q;
 reg start;
 reg [7:0] data;
 wire busy,tx_done;
 assign frame_done=phase==WAIT_TX && tx_done && byte_index==FRAME_BYTES-1;
 reg [19:0] adjusted;
 wire [19:0] next_bcd={adjusted[18:0],binary[15]};
 integer i;
 always @(*)begin
 adjusted=bcd;
 for(i=0;i<5;i=i+1)if(bcd[4*i +: 4]>=5)adjusted[4*i +: 4]=bcd[4*i +: 4]+4'd3;
 end
 function [7:0] hex_ascii;
 input [3:0] nibble;
 begin hex_ascii=nibble<10 ? 8'd48+{4'd0,nibble} : 8'd55+{4'd0,nibble}; end
 endfunction
 function [7:0] frame_byte;
 input [8:0] index;
 reg [8:0] offset;
 begin
 if(index>=ROUTE_BYTES+69)begin
 offset=index-(ROUTE_BYTES+69);
 case(offset)
 0:frame_byte=8'd32;
 1:frame_byte=8'd83;
 2:frame_byte=8'd69;
 3:frame_byte=8'd61;
 4:frame_byte=hex_ascii({1'b0,sample_error_q});
 5:frame_byte=8'd32;
 6:frame_byte=8'd70;
 7:frame_byte=8'd69;
 8:frame_byte=8'd61;
 9:frame_byte=hex_ascii({1'b0,fault_sample_error_q});
 10:frame_byte=8'd13;
 11:frame_byte=8'd10;
 default:frame_byte=8'd63;
 endcase
 end else if(index>=ROUTE_BYTES-2)begin
 offset=index-(ROUTE_BYTES-2);
 case(offset)
 0:frame_byte=8'd32;
 1:frame_byte=8'd72;
 2:frame_byte=8'd73;
 3:frame_byte=8'd61;
 4:frame_byte=hex_ascii({2'd0,history_index_q[5:4]});
 5:frame_byte=hex_ascii(history_index_q[3:0]);
 6:frame_byte=8'd32;
 7:frame_byte=8'd72;
 8:frame_byte=8'd67;
 9:frame_byte=8'd61;
 10:frame_byte=hex_ascii({1'd0,history_count_q[6:4]});
 11:frame_byte=hex_ascii(history_count_q[3:0]);
 12:frame_byte=8'd32;
 13:frame_byte=8'd72;
 14:frame_byte=8'd70;
 15:frame_byte=8'd61;
 16:frame_byte=hex_ascii({3'd0,history_frozen_q});
 17:frame_byte=8'd32;
 18:frame_byte=8'd72;
 19:frame_byte=8'd68;
 20:frame_byte=8'd61;
 21:frame_byte=hex_ascii(history_word_q[196 +: 4]);
 22:frame_byte=hex_ascii(history_word_q[192 +: 4]);
 23:frame_byte=hex_ascii(history_word_q[188 +: 4]);
 24:frame_byte=hex_ascii(history_word_q[184 +: 4]);
 25:frame_byte=hex_ascii(history_word_q[180 +: 4]);
 26:frame_byte=hex_ascii(history_word_q[176 +: 4]);
 27:frame_byte=hex_ascii(history_word_q[172 +: 4]);
 28:frame_byte=hex_ascii(history_word_q[168 +: 4]);
 29:frame_byte=hex_ascii(history_word_q[164 +: 4]);
 30:frame_byte=hex_ascii(history_word_q[160 +: 4]);
 31:frame_byte=hex_ascii(history_word_q[156 +: 4]);
 32:frame_byte=hex_ascii(history_word_q[152 +: 4]);
 33:frame_byte=hex_ascii(history_word_q[148 +: 4]);
 34:frame_byte=hex_ascii(history_word_q[144 +: 4]);
 35:frame_byte=hex_ascii(history_word_q[140 +: 4]);
 36:frame_byte=hex_ascii(history_word_q[136 +: 4]);
 37:frame_byte=hex_ascii(history_word_q[132 +: 4]);
 38:frame_byte=hex_ascii(history_word_q[128 +: 4]);
 39:frame_byte=hex_ascii(history_word_q[124 +: 4]);
 40:frame_byte=hex_ascii(history_word_q[120 +: 4]);
 41:frame_byte=hex_ascii(history_word_q[116 +: 4]);
 42:frame_byte=hex_ascii(history_word_q[112 +: 4]);
 43:frame_byte=hex_ascii(history_word_q[108 +: 4]);
 44:frame_byte=hex_ascii(history_word_q[104 +: 4]);
 45:frame_byte=hex_ascii(history_word_q[100 +: 4]);
 46:frame_byte=hex_ascii(history_word_q[96 +: 4]);
 47:frame_byte=hex_ascii(history_word_q[92 +: 4]);
 48:frame_byte=hex_ascii(history_word_q[88 +: 4]);
 49:frame_byte=hex_ascii(history_word_q[84 +: 4]);
 50:frame_byte=hex_ascii(history_word_q[80 +: 4]);
 51:frame_byte=hex_ascii(history_word_q[76 +: 4]);
 52:frame_byte=hex_ascii(history_word_q[72 +: 4]);
 53:frame_byte=hex_ascii(history_word_q[68 +: 4]);
 54:frame_byte=hex_ascii(history_word_q[64 +: 4]);
 55:frame_byte=hex_ascii(history_word_q[60 +: 4]);
 56:frame_byte=hex_ascii(history_word_q[56 +: 4]);
 57:frame_byte=hex_ascii(history_word_q[52 +: 4]);
 58:frame_byte=hex_ascii(history_word_q[48 +: 4]);
 59:frame_byte=hex_ascii(history_word_q[44 +: 4]);
 60:frame_byte=hex_ascii(history_word_q[40 +: 4]);
 61:frame_byte=hex_ascii(history_word_q[36 +: 4]);
 62:frame_byte=hex_ascii(history_word_q[32 +: 4]);
 63:frame_byte=hex_ascii(history_word_q[28 +: 4]);
 64:frame_byte=hex_ascii(history_word_q[24 +: 4]);
 65:frame_byte=hex_ascii(history_word_q[20 +: 4]);
 66:frame_byte=hex_ascii(history_word_q[16 +: 4]);
 67:frame_byte=hex_ascii(history_word_q[12 +: 4]);
 68:frame_byte=hex_ascii(history_word_q[8 +: 4]);
 69:frame_byte=hex_ascii(history_word_q[4 +: 4]);
 70:frame_byte=hex_ascii(history_word_q[0 +: 4]);
 71:frame_byte=13; 72:frame_byte=10; default:frame_byte=63;
 endcase
 end else if(index>=NORMAL_BYTES-2)begin
 offset=index-(NORMAL_BYTES-2);
 case(offset)
 0:frame_byte=8'd32;
 1:frame_byte=8'd82;
 2:frame_byte=8'd67;
 3:frame_byte=8'd61;
 4:frame_byte=hex_ascii({1'd0,diagnostic_q[63 +: 3]});
 5:frame_byte=8'd32;
 6:frame_byte=8'd82;
 7:frame_byte=8'd83;
 8:frame_byte=8'd61;
 9:frame_byte=hex_ascii(diagnostic_q[59 +: 4]);
 10:frame_byte=8'd32;
 11:frame_byte=8'd83;
 12:frame_byte=8'd71;
 13:frame_byte=8'd61;
 14:frame_byte=hex_ascii({1'd0,diagnostic_q[56 +: 3]});
 15:frame_byte=8'd32;
 16:frame_byte=8'd70;
 17:frame_byte=8'd67;
 18:frame_byte=8'd61;
 19:frame_byte=hex_ascii(diagnostic_q[52 +: 4]);
 20:frame_byte=hex_ascii(diagnostic_q[48 +: 4]);
 21:frame_byte=8'd32;
 22:frame_byte=8'd70;
 23:frame_byte=8'd77;
 24:frame_byte=8'd61;
 25:frame_byte=hex_ascii(diagnostic_q[44 +: 4]);
 26:frame_byte=hex_ascii(diagnostic_q[40 +: 4]);
 27:frame_byte=hex_ascii(diagnostic_q[36 +: 4]);
 28:frame_byte=hex_ascii(diagnostic_q[32 +: 4]);
 29:frame_byte=8'd32;
 30:frame_byte=8'd70;
 31:frame_byte=8'd80;
 32:frame_byte=8'd61;
 33:frame_byte=hex_ascii(diagnostic_q[28 +: 4]);
 34:frame_byte=hex_ascii(diagnostic_q[24 +: 4]);
 35:frame_byte=hex_ascii(diagnostic_q[20 +: 4]);
 36:frame_byte=hex_ascii(diagnostic_q[16 +: 4]);
 37:frame_byte=hex_ascii(diagnostic_q[12 +: 4]);
 38:frame_byte=hex_ascii(diagnostic_q[8 +: 4]);
 39:frame_byte=hex_ascii(diagnostic_q[4 +: 4]);
 40:frame_byte=hex_ascii(diagnostic_q[0 +: 4]);
 41:frame_byte=8'd32;
 42:frame_byte=8'd68;
 43:frame_byte=8'd86;
 44:frame_byte=8'd61;
 45:frame_byte=hex_ascii({3'd0,diagnostic_q[66 +: 1]});
 46:frame_byte=13; 47:frame_byte=10; default:frame_byte=63;
 endcase
 end else if(index>=OLD_BYTES-2)begin
 offset=index-(OLD_BYTES-2);
 case(offset)
 0:frame_byte=8'd32;
 1:frame_byte=8'd73;
 2:frame_byte=8'd86;
 3:frame_byte=8'd61;
 4:frame_byte=8'd48+{7'd0,imu_q};
 5:frame_byte=8'd32;
 6:frame_byte=8'd82;
 7:frame_byte=8'd81;
 8:frame_byte=8'd61;
 9:frame_byte=hex_ascii(roll_q[28 +: 4]);
 10:frame_byte=hex_ascii(roll_q[24 +: 4]);
 11:frame_byte=hex_ascii(roll_q[20 +: 4]);
 12:frame_byte=hex_ascii(roll_q[16 +: 4]);
 13:frame_byte=hex_ascii(roll_q[12 +: 4]);
 14:frame_byte=hex_ascii(roll_q[8 +: 4]);
 15:frame_byte=hex_ascii(roll_q[4 +: 4]);
 16:frame_byte=hex_ascii(roll_q[0 +: 4]);
 17:frame_byte=8'd32;
 18:frame_byte=8'd80;
 19:frame_byte=8'd81;
 20:frame_byte=8'd61;
 21:frame_byte=hex_ascii(pitch_q[28 +: 4]);
 22:frame_byte=hex_ascii(pitch_q[24 +: 4]);
 23:frame_byte=hex_ascii(pitch_q[20 +: 4]);
 24:frame_byte=hex_ascii(pitch_q[16 +: 4]);
 25:frame_byte=hex_ascii(pitch_q[12 +: 4]);
 26:frame_byte=hex_ascii(pitch_q[8 +: 4]);
 27:frame_byte=hex_ascii(pitch_q[4 +: 4]);
 28:frame_byte=hex_ascii(pitch_q[0 +: 4]);
 29:frame_byte=8'd32;
 30:frame_byte=8'd89;
 31:frame_byte=8'd81;
 32:frame_byte=8'd61;
 33:frame_byte=hex_ascii(yaw_q[28 +: 4]);
 34:frame_byte=hex_ascii(yaw_q[24 +: 4]);
 35:frame_byte=hex_ascii(yaw_q[20 +: 4]);
 36:frame_byte=hex_ascii(yaw_q[16 +: 4]);
 37:frame_byte=hex_ascii(yaw_q[12 +: 4]);
 38:frame_byte=hex_ascii(yaw_q[8 +: 4]);
 39:frame_byte=hex_ascii(yaw_q[4 +: 4]);
 40:frame_byte=hex_ascii(yaw_q[0 +: 4]);
 41:frame_byte=13;42:frame_byte=10;default:frame_byte=63;
 endcase
 end else if(ERROR_REPORT!=0 && index>=BASE_BYTES-2)begin
 offset=index-ERROR_OFFSET;
 case(offset)
 0,6,12,18,24:frame_byte=8'd32;
 1,7,13,19:frame_byte=8'd69;
 2:frame_byte=8'd49;
 8:frame_byte=8'd50;
 14:frame_byte=8'd51;
 20:frame_byte=8'd52;
 3,9,15,21,27:frame_byte=8'd61;
 4:frame_byte=hex_ascii(errors_q[4 +: 4]);
 5:frame_byte=hex_ascii(errors_q[0 +: 4]);
 10:frame_byte=hex_ascii(errors_q[12 +: 4]);
 11:frame_byte=hex_ascii(errors_q[8 +: 4]);
 16:frame_byte=hex_ascii(errors_q[20 +: 4]);
 17:frame_byte=hex_ascii(errors_q[16 +: 4]);
 22:frame_byte=hex_ascii(errors_q[28 +: 4]);
 23:frame_byte=hex_ascii(errors_q[24 +: 4]);
 25:frame_byte=8'd70;
 26:frame_byte=8'd87;
 28:frame_byte=hex_ascii(fault_wheels_q);
 29:frame_byte=8'd13;
 30:frame_byte=8'd10;
 default:frame_byte=8'd63;
 endcase
 end else case(index)
            7'd0: frame_byte=8'd78;
            7'd1: frame_byte=8'd61;
            7'd2: frame_byte=8'd48+SENSOR_COUNT;
            7'd3: frame_byte=8'd32;
            7'd4: frame_byte=8'd70;
            7'd5: frame_byte=8'd61;
            7'd6: frame_byte=8'd48+{4'd0,digits[16 +: 4]};
            7'd7: frame_byte=8'd48+{4'd0,digits[12 +: 4]};
            7'd8: frame_byte=8'd48+{4'd0,digits[8 +: 4]};
            7'd9: frame_byte=8'd48+{4'd0,digits[4 +: 4]};
            7'd10: frame_byte=8'd48+{4'd0,digits[0 +: 4]};
            7'd11: frame_byte=8'd32;
            7'd12: frame_byte=8'd76;
            7'd13: frame_byte=8'd61;
            7'd14: frame_byte=8'd48+{4'd0,digits[36 +: 4]};
            7'd15: frame_byte=8'd48+{4'd0,digits[32 +: 4]};
            7'd16: frame_byte=8'd48+{4'd0,digits[28 +: 4]};
            7'd17: frame_byte=8'd48+{4'd0,digits[24 +: 4]};
            7'd18: frame_byte=8'd48+{4'd0,digits[20 +: 4]};
            7'd19: frame_byte=8'd32;
            7'd20: frame_byte=8'd82;
            7'd21: frame_byte=8'd61;
            7'd22: frame_byte=8'd48+{4'd0,digits[56 +: 4]};
            7'd23: frame_byte=8'd48+{4'd0,digits[52 +: 4]};
            7'd24: frame_byte=8'd48+{4'd0,digits[48 +: 4]};
            7'd25: frame_byte=8'd48+{4'd0,digits[44 +: 4]};
            7'd26: frame_byte=8'd48+{4'd0,digits[40 +: 4]};
            7'd27: frame_byte=8'd32;
            7'd28: frame_byte=8'd83;
            7'd29: frame_byte=8'd70;
            7'd30: frame_byte=8'd61;
            7'd31: frame_byte=8'd48+{5'd0,flags[2:0]};
            7'd32: frame_byte=8'd32;
            7'd33: frame_byte=8'd76;
            7'd34: frame_byte=8'd86;
            7'd35: frame_byte=8'd61;
            7'd36: frame_byte=8'd48+{6'd0,flags[4:3]};
            7'd37: frame_byte=8'd32;
            7'd38: frame_byte=8'd83;
            7'd39: frame_byte=8'd84;
            7'd40: frame_byte=8'd61;
            7'd41: frame_byte=state_q<10 ? 8'd48+{4'd0,state_q} : 8'd55+{4'd0,state_q};
            7'd42: frame_byte=8'd32;
            7'd43: frame_byte=8'd77;
            7'd44: frame_byte=8'd70;
            7'd45: frame_byte=8'd61;
            7'd46: frame_byte=8'd48+{5'd0,motor_q};
            7'd47: frame_byte=8'd32;
            7'd48: frame_byte=8'd65;
            7'd49: frame_byte=8'd82;
            7'd50: frame_byte=8'd77;
            7'd51: frame_byte=8'd61;
            7'd52: frame_byte=8'd48+{7'd0,flags[5]};
            7'd53: frame_byte=8'd32;
            7'd54: frame_byte=8'd82;
            7'd55: frame_byte=8'd85;
            7'd56: frame_byte=8'd78;
            7'd57: frame_byte=8'd61;
            7'd58: frame_byte=8'd48+{7'd0,flags[6]};
            7'd59: frame_byte=8'd32;
            7'd60: frame_byte=8'd68;
            7'd61: frame_byte=8'd82;
            7'd62: frame_byte=8'd86;
            7'd63: frame_byte=8'd61;
            7'd64: frame_byte=8'd48+{7'd0,flags[7]};
            7'd65: frame_byte=8'd32;
            7'd66: frame_byte=8'd69;
            7'd67: frame_byte=8'd83;
            7'd68: frame_byte=8'd84;
            7'd69: frame_byte=8'd61;
            7'd70: frame_byte=8'd48+{7'd0,flags[8]};
            7'd71: frame_byte=8'd32;
            7'd72: frame_byte=8'd70;
            7'd73: frame_byte=8'd76;
            7'd74: frame_byte=8'd84;
            7'd75: frame_byte=8'd61;
            7'd76: frame_byte=8'd48+{7'd0,flags[9]};
            8'd77: frame_byte=ENCODER_REPORT ? 8'd32 : 8'd13;
            8'd78: frame_byte=ENCODER_REPORT ? 8'd80 : 8'd10;
            8'd79: frame_byte=ENCODER_REPORT ? 8'd49 : 8'd63;
            8'd80: frame_byte=ENCODER_REPORT ? 8'd61 : 8'd63;
            8'd81: frame_byte=hex_ascii(positions_q[28 +: 4]);
            8'd82: frame_byte=hex_ascii(positions_q[24 +: 4]);
            8'd83: frame_byte=hex_ascii(positions_q[20 +: 4]);
            8'd84: frame_byte=hex_ascii(positions_q[16 +: 4]);
            8'd85: frame_byte=hex_ascii(positions_q[12 +: 4]);
            8'd86: frame_byte=hex_ascii(positions_q[8 +: 4]);
            8'd87: frame_byte=hex_ascii(positions_q[4 +: 4]);
            8'd88: frame_byte=hex_ascii(positions_q[0 +: 4]);
            8'd89: frame_byte=ENCODER_REPORT ? 8'd32 : 8'd63;
            8'd90: frame_byte=ENCODER_REPORT ? 8'd80 : 8'd63;
            8'd91: frame_byte=ENCODER_REPORT ? 8'd50 : 8'd63;
            8'd92: frame_byte=ENCODER_REPORT ? 8'd61 : 8'd63;
            8'd93: frame_byte=hex_ascii(positions_q[60 +: 4]);
            8'd94: frame_byte=hex_ascii(positions_q[56 +: 4]);
            8'd95: frame_byte=hex_ascii(positions_q[52 +: 4]);
            8'd96: frame_byte=hex_ascii(positions_q[48 +: 4]);
            8'd97: frame_byte=hex_ascii(positions_q[44 +: 4]);
            8'd98: frame_byte=hex_ascii(positions_q[40 +: 4]);
            8'd99: frame_byte=hex_ascii(positions_q[36 +: 4]);
            8'd100: frame_byte=hex_ascii(positions_q[32 +: 4]);
            8'd101: frame_byte=ENCODER_REPORT ? 8'd32 : 8'd63;
            8'd102: frame_byte=ENCODER_REPORT ? 8'd80 : 8'd63;
            8'd103: frame_byte=ENCODER_REPORT ? 8'd51 : 8'd63;
            8'd104: frame_byte=ENCODER_REPORT ? 8'd61 : 8'd63;
            8'd105: frame_byte=hex_ascii(positions_q[92 +: 4]);
            8'd106: frame_byte=hex_ascii(positions_q[88 +: 4]);
            8'd107: frame_byte=hex_ascii(positions_q[84 +: 4]);
            8'd108: frame_byte=hex_ascii(positions_q[80 +: 4]);
            8'd109: frame_byte=hex_ascii(positions_q[76 +: 4]);
            8'd110: frame_byte=hex_ascii(positions_q[72 +: 4]);
            8'd111: frame_byte=hex_ascii(positions_q[68 +: 4]);
            8'd112: frame_byte=hex_ascii(positions_q[64 +: 4]);
            8'd113: frame_byte=ENCODER_REPORT ? 8'd32 : 8'd63;
            8'd114: frame_byte=ENCODER_REPORT ? 8'd80 : 8'd63;
            8'd115: frame_byte=ENCODER_REPORT ? 8'd52 : 8'd63;
            8'd116: frame_byte=ENCODER_REPORT ? 8'd61 : 8'd63;
            8'd117: frame_byte=hex_ascii(positions_q[124 +: 4]);
            8'd118: frame_byte=hex_ascii(positions_q[120 +: 4]);
            8'd119: frame_byte=hex_ascii(positions_q[116 +: 4]);
            8'd120: frame_byte=hex_ascii(positions_q[112 +: 4]);
            8'd121: frame_byte=hex_ascii(positions_q[108 +: 4]);
            8'd122: frame_byte=hex_ascii(positions_q[104 +: 4]);
            8'd123: frame_byte=hex_ascii(positions_q[100 +: 4]);
            8'd124: frame_byte=hex_ascii(positions_q[96 +: 4]);
            8'd125: frame_byte=8'd32;
            8'd126: frame_byte=8'd86;
            8'd127: frame_byte=8'd49;
            8'd128: frame_byte=8'd61;
            8'd129: frame_byte=hex_ascii(velocities_q[12 +: 4]);
            8'd130: frame_byte=hex_ascii(velocities_q[8 +: 4]);
            8'd131: frame_byte=hex_ascii(velocities_q[4 +: 4]);
            8'd132: frame_byte=hex_ascii(velocities_q[0 +: 4]);
            8'd133: frame_byte=8'd32;
            8'd134: frame_byte=8'd86;
            8'd135: frame_byte=8'd50;
            8'd136: frame_byte=8'd61;
            8'd137: frame_byte=hex_ascii(velocities_q[28 +: 4]);
            8'd138: frame_byte=hex_ascii(velocities_q[24 +: 4]);
            8'd139: frame_byte=hex_ascii(velocities_q[20 +: 4]);
            8'd140: frame_byte=hex_ascii(velocities_q[16 +: 4]);
            8'd141: frame_byte=8'd32;
            8'd142: frame_byte=8'd86;
            8'd143: frame_byte=8'd51;
            8'd144: frame_byte=8'd61;
            8'd145: frame_byte=hex_ascii(velocities_q[44 +: 4]);
            8'd146: frame_byte=hex_ascii(velocities_q[40 +: 4]);
            8'd147: frame_byte=hex_ascii(velocities_q[36 +: 4]);
            8'd148: frame_byte=hex_ascii(velocities_q[32 +: 4]);
            8'd149: frame_byte=8'd32;
            8'd150: frame_byte=8'd86;
            8'd151: frame_byte=8'd52;
            8'd152: frame_byte=8'd61;
            8'd153: frame_byte=hex_ascii(velocities_q[60 +: 4]);
            8'd154: frame_byte=hex_ascii(velocities_q[56 +: 4]);
            8'd155: frame_byte=hex_ascii(velocities_q[52 +: 4]);
            8'd156: frame_byte=hex_ascii(velocities_q[48 +: 4]);
            8'd157: frame_byte=8'd13;
            8'd158: frame_byte=8'd10;
 default:frame_byte=8'd63;
 endcase end
 endfunction
 task3_uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) u_tx(
 .clk(clk),.rst_n(rst_n),.start(start),.data(data),.tx(tx),.busy(busy),.done(tx_done));
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin sample_error_q<=0;fault_sample_error_q<=0;history_index_q<=0;history_count_q<=0;history_frozen_q<=0;history_word_q<=0;diagnostic_q<=0;imu_q<=0;roll_q<=0;pitch_q<=0;yaw_q<=0;phase<=IDLE;period_count<=0;values<=0;digits<=0;bcd<=0;binary<=0;
 convert_bit<=0;convert_channel<=0;flags<=0;state_q<=0;motor_q<=0;positions_q<=0;velocities_q<=0;errors_q<=0;fault_wheels_q<=0;byte_index<=0;start<=0;data<=0;end
 else begin
 start<=0;
 if(period_count<FRAME_CYCLES-1)period_count<=period_count+1'b1;
 case(phase)
 IDLE:if(period_count>=FRAME_CYCLES-1)begin
 sample_error_q<=sample_error;fault_sample_error_q<=fault_sample_error;history_index_q<=history_index;history_count_q<=history_count;history_frozen_q<=history_frozen;history_word_q<=history_word;diagnostic_q<=route_diagnostic;imu_q<=imu_valid;roll_q<=roll_q16;pitch_q<=pitch_q16;yaw_q<=yaw_q16;
 period_count<=0;values<={right_mm,left_mm,front_mm};binary<=front_mm;bcd<=0;
 convert_bit<=0;convert_channel<=0;
 flags<={fault,emergency_stop,drive_enabled,run_request,armed,speed_level,sensor_fault};
 state_q<=route_state;motor_q<=motor_fault_code;
 positions_q<=positions;velocities_q<=velocities;errors_q<=encoder_errors;fault_wheels_q<=fault_wheels;phase<=CONVERT;end
 CONVERT:begin
 bcd<=next_bcd;binary<={binary[14:0],1'b0};
 if(convert_bit==15)begin
 digits[20*convert_channel +: 20]<=next_bcd;
 if(convert_channel==2)begin byte_index<=0;phase<=ISSUE;end
 else begin convert_channel<=convert_channel+1'b1;convert_bit<=0;bcd<=0;
 binary<=convert_channel==0?values[31:16]:values[47:32];end
 end else convert_bit<=convert_bit+1'b1;
 end
 ISSUE:if(!busy)begin data<=frame_byte(byte_index);start<=1;phase<=WAIT_TX;end
 WAIT_TX:if(tx_done)begin
 if(byte_index==FRAME_BYTES-1)phase<=IDLE;
 else begin byte_index<=byte_index+1'b1;phase<=ISSUE;end
 end
 default:phase<=IDLE;
 endcase end
endmodule

