# 作品源代码说明

**作品名称**：基于 FPGA 的具身智能机器人实时感知与自主控制
**队伍名称**：巅峰第一队
**队伍编号**：48347
**提交日期**：2026-11-02

---

## 一、代码版本

本压缩包内的工程源码与下述位流**逐文件对应**，可复现编译结果：

| 项 | 值 |
|---|---|
| 源码版本 | **v91**（2026-10-31 定版） |
| 位流文件 | `04_位流_v91/v91.fs`，36,647,418 字节 |
| 位流 MD5 | `da2ae90b3f8a6cd6ddea5dbe440ea09e` |
| 综合/布线工具 | 高云云源 Gowin V1.9.12（x64） |
| 目标器件 | GW5AT-138B（GW5AT-LV138PG484AC1/I0） |
| 综合顶层模块 | **`ov5640_ddr3_tft`**（`01_主工程_robot/src/ov5640_ddr3_tft.v`） |
| 主时钟 | 50 MHz（晶振 Y18） |

工程源码取自版本管理系统中 v91 提交对应的完整快照，非工作目录的临时状态。关键文件的 MD5 见同级目录 `校验_MD5.txt`。

---

## 二、目录结构

```
代码提交_v91_20261102/
├─ README_代码说明.md          本文件
├─ 校验_MD5.txt                关键文件与位流的 MD5
├─ 01_主工程_robot/            ★ 可编译工程（核心）
│   ├─ robot.gprj              高云工程文件，双击用 Gowin IDE 打开
│   └─ src/                    全部 RTL 源码、IP 核配置与约束文件
├─ 02_上位机/                  Web Bluetooth 上位机（单文件 HTML）
│   ├─ index.html              导航/避障/遥测控制台（任务一~四）
│   ├─ 控制台_任务五六.html    视觉任务控制台（任务五/六）
│   └─ *.bat                   本机启动脚本（起本地 HTTP 服务）
├─ 03_仿真验证/testbench/      20 个 ModelSim 测试平台（不参与综合）
└─ 04_位流_v91/                v91.fs 与对应时序报告
```

---

## 三、编译方法

1. 用 Gowin V1.9.12 打开 `01_主工程_robot/robot.gprj`
2. 确认器件为 `GW5AT-LV138PG484AC1/I0`（工程文件已自带，通常无需手改）
3. 确认综合顶层为 `ov5640_ddr3_tft`
4. 依次执行 Synthesis → Place & Route → Generate Bitstream

**注意事项**

- `src/impl/` 目录未包含在压缩包内，首次打开工程时 IDE 会自动重新生成。
- 约束文件 `src/ov5640_ddr3_tft.cst`（引脚绑定）与 `src/ov5640_ddr3_tft.sdc`（时序约束）均已列入工程 FileList，**请勿修改引脚定义**——引脚与 ACG720 开发板的物理接线一一对应。
- `src/` 下含 10 个 `.ipc` 文件，为 PLL / DDR3 等 IP 核的配置，重新生成 IP 时需保留。
- 工程 FileList 中 `enable="0"` 的 4 个文件（`gowin_clkdiv.v`、`dvi_encoder.v`、`dvi_tx.v`、`gowin_rpll.v`）为参考工程遗留，本作品未使用，保留以维持工程完整性。

---

## 四、模块构成

### 4.1 图像链路（任务一）

| 模块 | 路径 | 说明 |
|---|---|---|
| OV5640 配置 | `src/camera_init/` | 上电寄存器初始化序列 |
| 像素捕获 | `src/DVP_Capture/` | DVP 并口时序解析 |
| 帧缓存 | `src/ddr3_ctrl_2port/`、`src/gowin_ddr/` | 双端口 DDR3 读写仲裁 |
| 显示输出 | `src/vga/`、`src/tft_pll/` | 800×480 时序发生器 |
| 时钟 | `src/camera_pll/`、`src/ddr_pll/` | 像素时钟与 DDR3 时钟 |

### 4.2 运动控制与导航（任务二、任务四）

| 模块 | 路径 | 说明 |
|---|---|---|
| 四轮闭环控制 | `src/task2/selftest_ir.v` | 编码器测速 + Q8 定点 PI，任务二链 |
| 整合链（任务一~三） | `src/task3/task123_*.v`、`task3_*.v` | 任务二/三的板级整合、避障决策、运动基元、UART 遥测 |
| 实时避障 | `src/task3/obstacle_decision.v`、`src/own/dist_filter.v` | 超声波测距中值滤波与分档决策 |
| 导航决策 | `src/own/nav_ctrl.v` | 航点队列、两阶段"对准—直行"、避障绕行 FSM |
| 航点队列 | `src/own/wp_queue.v` | 16 点航点存储与索引 |
| 电机输出 | `src/own/nav_motor.v`、`mech_ik.v`、`speed_loop.v`、`pwm_gen.v` | 麦克纳姆轮逆运动学 + 四路 PWM 生成 |
| 里程计 | `src/own/odom_core.v`、`odom_top.v` | 四轮编码器解算位姿，IMU 增量融合航向 |
| 姿态解算 | `src/own/imu_attitude.v`、`src/own/mpu6050_ctrl.v` | 陀螺积分 + 互补滤波，输出 Q16.16 角度 |
| 方位角 | `src/own/cordic_atan2.v` | 流水线 CORDIC 求目标方位 |
| 控制权仲裁 | `src/ov5640_ddr3_tft.v`（`motor_owner`） | 两套闭环共用一组电机的硬件二选一互斥 |

### 4.3 视觉识别与任务执行（任务五、任务六）

| 模块 | 路径 | 说明 |
|---|---|---|
| 颜色判据 | `src/vision/vision_front.v` | 像素域色度阈值分割，旁路叠加包围框 |
| 连通域统计 | `src/vision/vision_blob.v` | 目标包围框提取 |
| 安全限定 | `src/vision/vision_detector.v` | 检测结果有效性与超时保护 |
| 任务状态机 | `src/vision/vision_mission.v` | 搜索→对准→接近→到达/故障 六状态编排 |
| 指令接口 | `src/vision/vision_command.v` | `$FIND` / `$VSTOP` 指令解析 |

### 4.4 通信与显示

| 模块 | 路径 | 说明 |
|---|---|---|
| 蓝牙链路 | `src/own/bt_link.v` | ATK-BLE02 串口收发与链路语义 |
| 指令解码 | `src/own/bt_cmd2.v` | 13 条上位机指令的统一解码器 |
| 遥测帧 | `src/own/bt_telem2.v` | 43 字节位姿/速度/电量遥测 |
| OSD 叠加 | `src/own/osd_overlay.v`、`osd_num_writer.v` | 画面叠加 PCLK/帧率/横滚/俯仰 |
| 屏幕字模 | `src/own/font_rom_init.vh` | 5×7 点阵字符 ROM 初始化 |

---

## 五、设计要点（详见技术报告）

1. **六个任务共用一套位姿**：航向解算结果只有一份，同时供给 OSD、里程计、航向控制、遥测与顶层快照，避免多处不一致。
2. **两套闭环共用一组电机**：`motor_owner` 做硬件二选一互斥，导航接管时任务二链整链停手；物理按键 S3 走独立硬件通路，不经任何状态机。
3. **视觉判据挂在像素域旁路**：判据在 87 MHz 像素域计算，决策在 50 MHz 域执行，两级之间只交换一组快照寄存器，判据层数增加不影响显示帧率。
4. **控制周期恒定 10 ms**：所有控制逻辑用 RTL 实现，无软核、无操作系统，周期不随负载变化。

---

## 六、上位机

`02_上位机/` 下的两个 HTML 均为单文件、零依赖，通过 Web Bluetooth 与车载 ATK-BLE02 模块通信。

- **必须**通过 `https://` 或 `localhost` 打开（浏览器对 Web Bluetooth 的安全限制），因此附带 `.bat` 脚本用于启动本地 HTTP 服务。
- 不要直接双击 `index.html`（`file://` 协议下蓝牙不可用）。

---

## 七、仿真验证

`03_仿真验证/testbench/` 内含 20 个 ModelSim 测试平台，覆盖导航状态机、电机逆解、航点队列、里程计、IMU 姿态、蓝牙编解码等模块。这些文件仅用于仿真验证，**不在 Gowin 工程 FileList 中，不参与综合**。

测试平台引用的被测模块位于 `01_主工程_robot/src/` 下，编译仿真工程时请将该目录（及其子目录）加入 include 路径。

运行方式（ModelSim）：

```
vsim -c -voptargs="+acc" -do "run -all; quit -f" work.<tb_name>
```

---

## 八、联系方式

如有编译或复现问题，请通过赛事组委会联系队伍（巅峰第一队，编号 48347）。
