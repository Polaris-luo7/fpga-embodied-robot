//=============================================================================
// tb_wp_queue.v -- 途径点队列用例（push/pop/seek/clr/满/空 边界）
//=============================================================================
`timescale 1ns/1ps

module tb_wp_queue;
    reg clk=0, rst_n=0, clr=0, push=0, pop=0, seek_en=0;
    reg signed [31:0] px=0, py=0;
    reg [4:0] seek_n=0;
    wire signed [31:0] hx, hy;
    wire [4:0] count;
    wire empty, full;

    wp_queue #(.DEPTH(16)) dut (
        .clk(clk), .rst_n(rst_n), .clr(clr),
        .push(push), .push_x(px), .push_y(py),
        .pop(pop), .seek_en(seek_en), .seek_n(seek_n),
        .head_x(hx), .head_y(hy), .count(count), .empty(empty), .full(full)
    );

    always #10 clk = ~clk;

    integer i, fails=0;
    task push_pt(input signed [31:0] x, input signed [31:0] y);
        begin
            px = x; py = y; push = 1;
            @(posedge clk); push = 0;
            @(posedge clk);
        end
    endtask

    task chk(input [8*32-1:0] name, input [31:0] got, input [31:0] want);
        begin
            if (got === want) $display("[PASS] %0s = %0d", name, got);
            else begin fails = fails + 1;
                $display("[FAIL] %0s got=%0d want=%0d", name, got, want); end
        end
    endtask

    initial begin
        $display("=== tb_wp_queue start ===");
        repeat (5) @(posedge clk); rst_n = 1; repeat (2) @(posedge clk);
        chk("init_empty", {31'd0, empty}, 1);
        chk("init_count", {27'd0, count}, 0);

        // 压入 3 点
        push_pt(1500, 0); push_pt(1500, 1000); push_pt(0, 1000);
        chk("count3", {27'd0, count}, 3);
        chk("head_x",  hx, 1500);
        chk("head_y",  hy, 0);

        // seek 到第 2 点(索引 2 = 0-based) => 头应为 (0,1000), count=1
        seek_n = 5'd2; seek_en = 1;
        @(posedge clk); seek_en = 0; @(posedge clk);
        chk("seek_head_x", hx, 32'sd0);
        chk("seek_head_y", hy, 1000);
        chk("seek_count",  {27'd0, count}, 1);

        // pop 掉 => 空
        pop = 1; @(posedge clk); pop = 0; @(posedge clk);
        chk("after_pop_empty", {31'd0, empty}, 1);

        // 空队列 pop 不应出错(计数不越界)
        pop = 1; @(posedge clk); pop = 0; @(posedge clk);
        chk("empty_pop_safe", {27'd0, count}, 0);

        // 压满 16 点
        for (i = 0; i < 16; i = i + 1) push_pt(i*10, i*20);
        chk("full_count", {27'd0, count}, 16);
        chk("full_flag",  {31'd0, full}, 1);

        // 满时再 push 应被忽略(仍 16)
        push_pt(999, 999);
        chk("full_push_ignored", {27'd0, count}, 16);

        // clr 清空
        clr = 1; @(posedge clk); clr = 0; @(posedge clk);
        chk("clr_count", {27'd0, count}, 0);
        chk("clr_empty", {31'd0, empty}, 1);

        $display("=== tb_wp_queue done: fails=%0d ===", fails);
        $finish;
    end
endmodule
