// Distance policy only. Requests are not physical motion confirmation.
// Software example thresholds; require braking/layout calibration before use.
module obstacle_decision #(
    parameter integer SENSOR_COUNT = 3,
    parameter integer STALE_US = 600_000,
    parameter integer STOP_MM = 250,
    parameter integer STOP_RELEASE_MM = 350,
    parameter integer SLOW_MM = 600,
    parameter integer SLOW_RELEASE_MM = 700,
    parameter integer SIDE_CLEAR_MM = 600,
    parameter integer RECOVERY_SAMPLES = 3
)(
    input wire clk, rst_n, tick_us,
    input wire sample_strobe,
    input wire [1:0] sample_channel,
    input wire [15:0] sample_mm,
    input wire sample_valid,
    output reg [1:0] speed_level, // 0=stop, 1=slow, 2=normal
    output wire stop_req, slow_req,
    output reg [1:0] avoid_dir,  // 0=no suggestion, 1=left, 2=right
    output reg [2:0] sensor_fault,
    output wire [15:0] front_mm, left_mm, right_mm
);
    reg [15:0] distances [0:2];
    reg [31:0] age_us [0:2];
    reg [2:0] good;
    reg [31:0] recovery_count;
    integer i, j;
    wire front_bad = sensor_fault[0];
    wire left_clear = SENSOR_COUNT >= 2 && !sensor_fault[1] && distances[1] >= SIDE_CLEAR_MM;
    wire right_clear = SENSOR_COUNT >= 3 && !sensor_fault[2] && distances[2] >= SIDE_CLEAR_MM;
    assign front_mm = distances[0];
    assign left_mm = distances[1];
    assign right_mm = distances[2];
    assign stop_req = speed_level == 0;
    assign slow_req = speed_level == 1;

    always @(*) begin
        for (j=0; j<3; j=j+1)
            sensor_fault[j] = j < SENSOR_COUNT && (!good[j] || age_us[j] >= STALE_US);
        avoid_dir = 0;
        // Side data can suggest direction only when front data is trustworthy.
        if (stop_req && !front_bad && distances[0] <= STOP_MM) begin
            if (left_clear && right_clear)
                avoid_dir = (distances[1] >= distances[2]) ? 1 : 2;
            else if (left_clear) avoid_dir = 1;
            else if (right_clear) avoid_dir = 2;
        end
    end

    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin
            good <= 0; speed_level <= 0; recovery_count <= 0;
            for (i=0; i<3; i=i+1) begin
                distances[i] <= 0; age_us[i] <= STALE_US;
            end
        end else begin
            for (i=0; i<3; i=i+1)
                if (tick_us && age_us[i] < STALE_US) age_us[i] <= age_us[i]+1'b1;
            if (sample_strobe && sample_channel < SENSOR_COUNT) begin
                age_us[sample_channel] <= 0;
                good[sample_channel] <= sample_valid;
                distances[sample_channel] <= sample_valid ? sample_mm : 0;
            end
            if (front_bad) begin speed_level <= 0; recovery_count <= 0; end
            // New front data takes precedence over the previous stale flag.
            if (sample_strobe && sample_channel == 0) begin
                if (!sample_valid || sample_mm <= STOP_MM) begin
                    speed_level <= 0; recovery_count <= 0;
                end else if (speed_level == 0) begin
                    if (sample_mm >= STOP_RELEASE_MM) begin
                        if ((front_bad ? 0 : recovery_count) >= RECOVERY_SAMPLES-1) begin
                            speed_level <= (sample_mm >= SLOW_RELEASE_MM) ? 2 : 1;
                            recovery_count <= 0;
                        end else recovery_count <= front_bad ? 1 : recovery_count + 1'b1;
                    end else recovery_count <= 0;
                end else begin
                    recovery_count <= 0;
                    if (speed_level == 2 && sample_mm <= SLOW_MM) speed_level <= 1;
                    else if (speed_level == 1 && sample_mm >= SLOW_RELEASE_MM) speed_level <= 2;
                end
            end
        end
endmodule
