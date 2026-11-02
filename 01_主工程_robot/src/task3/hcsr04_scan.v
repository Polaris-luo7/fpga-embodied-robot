// Sequential HC-SR04 acquisition. No board-specific voltage or pins assumed.
// channel 0=front, 1=left, 2=right; SENSOR_COUNT may be 1..3.
module hcsr04_scan #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer SENSOR_COUNT = 3,
    parameter integer TRIG_US = 10,
    parameter integer SLOT_US = 65_000,
    parameter integer TIMEOUT_US = 30_000,
    parameter integer MIN_MM = 20,
    parameter integer MAX_MM = 4000
)(
    input wire clk, rst_n,
    input wire [SENSOR_COUNT-1:0] echo,
    output reg [SENSOR_COUNT-1:0] trig,
    output wire tick_us,
    output reg sample_strobe,
    output reg [1:0] sample_channel,
    output reg [15:0] sample_mm,
    output reg sample_valid,
    output reg [2:0] sample_error
);
    localparam integer US_DIV = CLK_HZ / 1_000_000;
    localparam [2:0] IDLE=0, TRIGGER=1, WAIT_RISE=2, MEASURE=3, GAP=4;
    reg [31:0] us_count;
    reg [SENSOR_COUNT-1:0] echo_meta, echo_sync;
    reg [2:0] state;
    reg [1:0] channel;
    reg [31:0] slot_count, wait_count, width_us;
    // Exact floor(width_us*5/29), updated alongside each measured microsecond.
    // Small remainder arithmetic replaces the long combinational divider.
    reg [31:0] converted_mm;
    reg [5:0] mm_remainder;
    wire [5:0] next_remainder=mm_remainder+6'd5;
    assign tick_us = (us_count == US_DIV-1);

    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            us_count <= 0;
            echo_meta <= 0; echo_sync <= 0;
        end else begin
            us_count <= tick_us ? 0 : us_count + 1'b1;
            echo_meta <= echo;
            echo_sync <= echo_meta;
        end

    task finish_sample;
        input good;
        input [2:0] error_code;
        input [15:0] distance;
        begin
            sample_strobe <= 1'b1;
            sample_channel <= channel;
            sample_valid <= good;
            sample_error <= error_code;
            sample_mm <= distance;
            trig <= 0;
            state <= GAP;
        end
    endtask

    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            trig <= 0; state <= IDLE; channel <= 0;
            slot_count <= 0; wait_count <= 0; width_us <= 0; converted_mm<=0;mm_remainder<=0;
            sample_strobe <= 0; sample_channel <= 0;
            sample_mm <= 0; sample_valid <= 0; sample_error <= 0;
        end else begin
            sample_strobe <= 0;
            if (tick_us) begin
                if (state != IDLE && slot_count < SLOT_US)
                    slot_count <= slot_count + 1'b1;
                case (state)
                    IDLE: begin
                        slot_count <= 0; wait_count <= 0; width_us <= 0; converted_mm<=0;mm_remainder<=0;
                        if (echo_sync[channel])
                            finish_sample(0, 3'd4, 0); // Echo already high
                        else begin
                            trig <= {{(SENSOR_COUNT-1){1'b0}},1'b1} << channel;
                            state <= TRIGGER;
                        end
                    end
                    TRIGGER: begin
                        if (wait_count == TRIG_US-1) begin
                            trig <= 0; wait_count <= 0; state <= WAIT_RISE;
                        end else wait_count <= wait_count + 1'b1;
                    end
                    WAIT_RISE: begin
                        if (echo_sync[channel]) begin
                            width_us <= 1;converted_mm<=0;mm_remainder<=5; wait_count <= 0; state <= MEASURE;
                        end else if (wait_count >= TIMEOUT_US-1)
                            finish_sample(0, 3'd1, 0); // no rising edge
                        else wait_count <= wait_count + 1'b1;
                    end
                    MEASURE: begin
                        if (!echo_sync[channel]) begin
                            if (converted_mm < MIN_MM || converted_mm > MAX_MM)
                                finish_sample(0, 3'd3, 0); // out of range
                            else finish_sample(1, 0, converted_mm[15:0]);
                        end else if (width_us >= TIMEOUT_US)
                            finish_sample(0, 3'd2, 0); // pulse stuck high
                        else begin
                            width_us <= width_us + 1'b1;
                            if (next_remainder>=6'd29) begin
                                converted_mm<=converted_mm+1'b1;
                                mm_remainder<=next_remainder-6'd29;
                            end else mm_remainder<=next_remainder;
                        end
                    end
                    GAP: begin
                        if (slot_count >= SLOT_US-1) begin
                            if (channel == SENSOR_COUNT-1) channel <= 2'd0;
                            else channel <= channel + 2'd1;
                            state <= IDLE;
                        end
                    end
                    default: begin trig <= 0; state <= IDLE; end
                endcase
            end
        end
endmodule
