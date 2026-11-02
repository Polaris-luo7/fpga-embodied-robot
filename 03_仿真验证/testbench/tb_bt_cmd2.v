//=============================================================================
// tb_bt_cmd2.v -- 指令解析器用例（16 项, 全部有硬期望值）
//   tb 用字符串直接构造帧, 不依赖 bt_frame(避免同错同错)
//=============================================================================
`timescale 1ns/1ps

module tb_bt_cmd2;
    localparam integer FRAME_MAX = 32;

    reg clk=0, rst_n=0, frame_vld=0;
    reg [FRAME_MAX*8-1:0] frame=0;
    reg [5:0] frame_len=0;
    wire [7:0] cmd;
    wire signed [31:0] arg0, arg1;
    wire cmd_vld;

    bt_cmd2 #(.FRAME_MAX(FRAME_MAX)) dut (
        .clk(clk), .rst_n(rst_n),
        .frame(frame), .frame_len(frame_len), .frame_vld(frame_vld),
        .cmd(cmd), .arg0(arg0), .arg1(arg1), .cmd_vld(cmd_vld)
    );

    always #10 clk = ~clk;

    // ---- 命令码 ----
    localparam [7:0] C_NONE=0, C_PING=1, C_STAT=2, C_CLR=3, C_ADD=4, C_ARM=5,
                     C_GO=6, C_STOP=7, C_ESTOP=8, C_WP=9, C_MV=10, C_GOP=11, C_TEL=12;

    integer j, fails=0, ncases=0;
    reg [8*32-1:0] sbuf;
    reg [7:0] ec;            // 期望命令码
    reg signed [31:0] e0, e1;

    task inject(input [8*32-1:0] s, input integer n);
        begin
            // ★ Verilog 字符串字面量**右对齐**: 首字符在最高有效字节
            //   s 宽 32 字节, 第 j 个字符(0-based) 在 s[8*(n-1-j) +: 8]
            frame = {(FRAME_MAX*8){1'b0}};
            for (j = 0; j < n; j = j + 1)
                frame[j*8 +: 8] = s[8*(n-1-j) +: 8];
            frame_len = n[5:0];
            @(posedge clk); frame_vld = 1;
            @(posedge clk); frame_vld = 0;
            repeat (n + 4) @(posedge clk);     // 等解析状态机走完
        end
    endtask

    task chk(input [8*40-1:0] name, input [7:0] gc, input signed [31:0] g0, input signed [31:0] g1);
        begin
            ncases = ncases + 1;
            if (gc === ec && g0 === e0 && g1 === e1)
                $display("[PASS] %0s cmd=%0d arg0=%0d arg1=%0d", name, gc, g0, g1);
            else begin
                fails = fails + 1;
                $display("[FAIL] %0s got cmd=%0d a0=%0d a1=%0d | want cmd=%0d a0=%0d a1=%0d",
                         name, gc, g0, g1, ec, e0, e1);
            end
        end
    endtask

    initial begin
        $display("=== tb_bt_cmd2 start ===");
        repeat (5) @(posedge clk); rst_n = 1; repeat (3) @(posedge clk);

        ec=C_PING;  e0=0;     e1=0;  inject("$PING", 5);            chk("PING",  cmd, arg0, arg1);
        ec=C_STAT;  e0=0;     e1=0;  inject("$STAT", 5);            chk("STAT",  cmd, arg0, arg1);
        ec=C_CLR;   e0=0;     e1=0;  inject("$CLR",  4);            chk("CLR",   cmd, arg0, arg1);
        ec=C_ARM;   e0=0;     e1=0;  inject("$ARM",  4);            chk("ARM",   cmd, arg0, arg1);
        ec=C_GO;    e0=0;     e1=0;  inject("$GO",   3);            chk("GO",    cmd, arg0, arg1);
        ec=C_STOP;  e0=0;     e1=0;  inject("$STOP", 5);            chk("STOP",  cmd, arg0, arg1);
        ec=C_ESTOP; e0=0;     e1=0;  inject("$ESTOP",6);            chk("ESTOP", cmd, arg0, arg1);

        ec=C_ADD;   e0=1500;  e1=1000; inject("$ADD,1500,1000", 14); chk("ADD+",  cmd, arg0, arg1);
        ec=C_ADD;   e0=-500;  e1=250;  inject("$ADD,-500,250",  13); chk("ADD-",  cmd, arg0, arg1);

        ec=C_WP;    e0=2;     e1=0;  inject("$WP,2", 5);            chk("WP",    cmd, arg0, arg1);
        ec=C_MV;    e0=0;     e1=1000; inject("$MV,FWD,1000", 12);  chk("MV_FWD",cmd, arg0, arg1);
        ec=C_MV;    e0=4;     e1=720;  inject("$MV,SPIN,720", 12);  chk("MV_SPIN",cmd,arg0, arg1);
        ec=C_MV;    e0=3;     e1=500;  inject("$MV,RIGHT,500",13);  chk("MV_RIGHT",cmd,arg0,arg1);

        ec=C_GOP;   e0=800;   e1=-45;  inject("$GOP,800,-45", 12);  chk("GOP",   cmd, arg0, arg1);
        ec=C_TEL;   e0=1;     e1=0;  inject("$TEL,1", 6);           chk("TEL",   cmd, arg0, arg1);

        ec=C_NONE;  e0=0;     e1=0;  inject("$XYZZY", 6);           chk("bad_cmd", cmd, arg0, arg1);
        ec=C_NONE;  e0=0;     e1=0;  inject("$PINGX", 6);           chk("PINGX",  cmd, arg0, arg1);

        $display("=== tb_bt_cmd2 done: cases=%0d fails=%0d ===", ncases, fails);
        $finish;
    end
endmodule
