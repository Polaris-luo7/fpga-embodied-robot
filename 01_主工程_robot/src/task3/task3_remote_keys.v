// Decode PLAY(0x43) and STOP(0x16) into virtual board-button presses.
// Direction/spin keys do not bypass autonomous task-three decisions.
module task3_remote_keys #(parameter integer PRESS_CYCLES=3_000_000)(
    input wire clk,rst_n,
    input wire [3:0] key,
    input wire [7:0] ir_cmd,
    input wire ir_valid,ir_repeat,
    output wire [3:0] effective_key
);
    reg [31:0] play_count,stop_count;
    assign effective_key=key & {stop_count==0,play_count==0,2'b11};
    always @(posedge clk or negedge rst_n)
        if(!rst_n) begin play_count<=0;stop_count<=0;end
        else begin
            if(play_count!=0) play_count<=play_count-1'b1;
            if(stop_count!=0) stop_count<=stop_count-1'b1;
            if(ir_valid && !ir_repeat && ir_cmd==8'h43 && play_count==0)
                play_count<=PRESS_CYCLES;
            if((ir_valid || ir_repeat) && ir_cmd==8'h16) begin
                stop_count<=PRESS_CYCLES;play_count<=0;
            end
        end
endmodule
