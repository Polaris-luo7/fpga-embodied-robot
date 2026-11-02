#=============================================================================
# ov5640_ddr3_tft.sdc -- experiment 2 (2026-10-02)
#-----------------------------------------------------------------------------
# Purpose : constrain the time domains that were NEVER constrained before.
#           A/B test proved: picture OK without OSD, garbage with OSD, and
#           removing cam_rate_meter did NOT fix it (exp1) => the deciding
#           variable is place&route itself, acting on an unconstrained design.
#
# Scope   : MINIMAL SET, 3 lines only (as specified).
#           - no create_generated_clock on purpose: let the tool derive all
#             PLL outputs (loc_clk50m / clk_200M / clk_tft / loc_clk24m).
#           - no .cst change.
#
# Syntax  : verified against a real, working Gowin project in this repo:
#             Robei_car/chuan_cheng/Gowinators/hw/src/TMDS_60HZ.sdc
#             Robei_car/chuan_cheng/Gowinators/hw/src/TMDS_800_600_60.sdc
#           Registering matters: Gowin does NOT auto-discover *.sdc.
#           It must be listed in the .gprj FileList as type="file.sdc",
#           enable="1".  (In that reference project 4 .sdc existed but only
#           ONE had enable="1".)
#
# Port names below are the REAL top-level port names of module
# gao_ov5640_ddr_tft (robot/src/ov5640_ddr3_tft.v):
#     input clk50M        (.cst: IO_LOC "clk50M" Y18)
#     input camera_pclk   (.cst: IO_LOC "camera_pclk" B1)
# NOTE: the reference design called it "cmos_pclk" -- that is THEIR name.
#       Do NOT copy that identifier here.
#
# period = 20ns (50 MHz) rationale for camera_pclk:
#     the real PCLK is not measured yet. From fps=31 the pixel rate is
#     approx 24 MHz; OV5640 commonly outputs 24/27/48 MHz.
#     20 ns is TIGHTER than the real value => over-constraining => the tool
#     works harder to protect that domain => favourable for the fix.
#     It is also far less aggressive than the reference design, which used
#     5.88 ns (170 MHz) for its pixel clock.
#     THIS IS AN EXPERIMENTAL VALUE, not a final one. Refine it once the
#     real pclk count is measured.
#
# Rollback: delete this file, or flip enable="1" to enable="0" in robot.gprj.
#=============================================================================

# --- system 50 MHz ---
create_clock -name clk50M -period 20 -waveform {0 10} [get_ports {clk50M}] -add

# --- camera PCLK : the core suspect (DVP capture domain) ---
# 2026-10-10 更新：PCLK 已在屏上实测 = 87 MHz => period = 1/87MHz = 11.49 ns
#   （原值 20 ns 是 10-02 未测 PCLK 时按 24 MHz 估的，属于欠约束）
create_clock -name camera_pclk -period 11.5 -waveform {0 5.75} [get_ports {camera_pclk}] -add

# --- the two domains are asynchronous: do not constrain them against
# --- each other, otherwise the cross-domain paths report false violations
set_clock_groups -asynchronous -group [get_clocks {clk50M}] -group [get_clocks {camera_pclk}]

#-----------------------------------------------------------------------------
# 2026-10-21 尝试过：clk50M 与 Gowin_PLL/CLKOUT1 声明异步组（消除 snapshot_cdc
# 邮箱假违例）——但自动派生时钟名在 SDC 加载阶段不存在（TA2004），此路不通。
# 结论：u_vel_cdc/u_imu_cdc/u_dist_cdc 上的违例为**慢速率信号假阳性**（10ms 粒度，
# 实测功能正常），保持记录在案；若实测出现途径点错乱再研究约束方案。
#-----------------------------------------------------------------------------
