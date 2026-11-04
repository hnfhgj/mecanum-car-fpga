# 麦轮车最终完整工程（2026-11-04）

最终组合：FPGA保护固件97F4 + Windows电脑控制台R6（包含任务3实时按键显示）。这不是97F5取消保护版。

## 入口

- 解压后双击“启动电脑控制台.cmd”。控制台源码是“电脑控制台/controller.py”。Python需包含Tkinter，依赖见该目录requirements.txt；启动脚本优先本机D盘Python313，其他电脑可安装依赖后用python controller.py运行。
- FPGA工程是“FPGA工程/motor/free_navigation.gprj”，顶层“free_navigation/free_navigation_top.v”。所有实际启用的.v、约束CST/SDC和ROM .hex均已完整保留原相对目录。
- 最终烧录文件是“烧录文件/最终固件_97F4.fs”，UserCode 000097F4。器件GW5AT-60B/PBGA484A，芯片GW5AT-LV60PG484AC1/I0，版本B，实际器件ID 0x0001481B。
- 在FPGA工程/motor目录运行Gowin的gw_sh.exe free_navigation/build.tcl可重新构建。当前已验证布局2/布线2，50MHz，setup和hold零违例。必须使用此工程入口，文件夹名含历史版本的依赖模块仍是本最终工程的正式组成部分。

## 功能与操作

任务3：启用后按住W/S前后、A/D纯横移，组合斜移；鼠标侧键1顺时针，侧键2逆时针，松开停车。实时按键方块及组合文字显示在遥控页。按键显示不改变控制条件。

任务4：车在矩形框内静止建系，核对外框后选目标并出发；保持原目标并根据障碍改道。即时方向保护沿用任务3，默认前后40cm、左右25cm，锥形半角约20.6度（可在界面调）；没有额外电脑短段/旋转空间停车否决。雷达失效、无线失联、持续定位失效、Esc和停止仍会停车。

选定通道保持执行；只有出口改变、方向阻挡等才重选。退出动作约320ms续发。无位移换向需要实际板端执行记录，避免仅凭电脑指令意图误判断无路。所有细节以电脑控制台/使用说明.md为准。

TFT、编码器、IMU姿态、雷达采集、摄像头及板载按键相关代码均包含。TOF用户已拆除，未接入；保留其原驱动和约束不代表实物可用。

## 烧录、接线和验证

当前板上已写入并重载读回97F4，见“验证记录/Flash实机烧录”。按既有要求，后续实际烧录前仍应通知现场用户。保留这份文件即可再次烧Flash，电脑控制台修改不必重烧FPGA。

接线以“说明/最终接线表.md”和工程当前CST为准。历史接线文件单独存为参考，不能覆盖最终轮序映射。FPGA输入输出的历史端口名与物理轮位并不一一同名，wheel_mapping/channel_map.v统一转换。

131项软件测试、六页界面冒烟通过；按键显示另验证单键、组合和松开不写运动状态。固件复用13项RTL仿真、56路约束核对和零时序违例记录。最新R6实车运动效果未由本次打包重新测试，不把离线模型视为实车验收。

软件回归：在电脑控制台目录运行python -m unittest test_field test_gui test_console test_inputs test_local test_r3 test_r5 test_r6；界面检查python controller.py --smoke。RTL仿真脚本为FPGA工程/motor/free_navigation/run_tests.ps1，需要本机Vivado xsim路径或修改binRoot。

本包包含最终启用工程、固件、控制台、测试/烧录记录、接线、资料和运行日志快照；不重复打入其他旧版完整工程、仿真缓存或安装器。
