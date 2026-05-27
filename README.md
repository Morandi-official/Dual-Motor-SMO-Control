# Dual-Motor-SMO-Control v4.0
本仓库用于备份 `dual_axis_servo_drive_fcl_qep_f2837x` 双电机工程中，相对最初原始工程新增或修改过的代码文件。`v4.0/` 目录只放代码文件，不放 CCS 工程元数据 `.project`、编译产物或完整 SDK 工程副本。

## eSMO 切入滑模闭环的具体过程

以 M1 为例，只有同时满足下面两个条件，才表示真正进入 eSMO 无感闭环：

```c
motorVars[0].positionFeedback == POSITION_FEEDBACK_ESMO
motorVars[0].ptrFCL->lsw == ENC_CALIBRATION_DONE
```

若 `M1_POSITION_FEEDBACK=POSITION_FEEDBACK_QEP`，`lsw=ENC_CALIBRATION_DONE` 表示 QEP 闭环；若为 `POSITION_FEEDBACK_ESMO_MONITOR`，控制仍由 QEP 完成，eSMO 只是旁路观测。

当前 M1 切入流程：

1. `ENC_ALIGNMENT`：对齐阶段，`IdRef_start=M1_STARTUP_ID_REF=0.20`，Iq 为 0。
2. `ENC_WAIT_FOR_INDEX`：eSMO 开环牵引阶段。eSMO 模式下禁止 CLA 因 QEP index 自动切闭环。开环目标 `M1_ESMO_FORCE_SPEED=0.08 pu`，最少强拖时间 `M1_ESMO_FORCE_RUN_SEC=1.0 s`。
3. 启动 Iq 控制：最大 `M1_STARTUP_IQ_REF=0.04 pu`。若 `esmoSpeedPu` 超过当前 `rc.SetpointValue`，`getSensorlessStartupIqRef()` 动态削弱 Iq，最低为 `0.04*0.10=0.004 pu`，超速带宽 `M1_STARTUP_OVERSPEED_BAND=0.03 pu`。
4. 接管判断：强拖时间满足后，要求 `rc.SetpointValue>=0.07 pu`、`esmoSpeedPu>=0.03 pu`、`Eq_mag>=0.055 pu`，且速度方向一致。满足后同步 eSMO 角度并置 `lsw=ENC_CALIBRATION_DONE`。
5. `ENC_CALIBRATION_DONE`：eSMO 提供电角度和速度，FCL Park 角度使用 `esmoAnglePu`，速度闭环接管 Iq。

最近一次 M1 测试：切换点约 `rcSetpoint=0.0739 pu`、`speedFbk=0.0862 pu`、`Eq_mag=0.0593 pu`；稳态约 `speed.Speed=0.1006 pu`、`esmoSpeedPu=0.0972 pu`、`tripFlagDMC=0`。

## 代码文件修改说明

- `include/dual_axis_servo_drive.h`：引入 sensorless 接口，保留原 QEP 路径。
- `include/dual_axis_servo_drive_sensorless.h`：新增双电机 eSMO 适配层接口，使用前向声明避免 `MOTOR_Vars_t` 包含顺序问题。
- `include/dual_axis_servo_drive_settings.h`：增加反馈源兼容定义，并将最终反馈选择转移到 CLA 安全的 `dual_axis_servo_drive_user.h`。
- `include/dual_axis_servo_drive_user.h`：增加 QEP/eSMO/eSMO monitor 选择宏；按电机参数 txt 修正 Rs、Ld、Lq、base current、base voltage、base flux、base frequency；加入 M1/M2 eSMO 和启动接管参数。
- `include/esmo.h`、`sources/esmo.c`：从单电机无感工程移植 eSMO 算法，适配当前工程 include path 和类型。
- `include/fcl_enum.h`：增加 `POSITION_FEEDBACK_*` 兼容宏，防止真实工程仍链接 SDK 原始头文件时报未定义。
- `libraries/fcl/include/fcl_cpu_cla_dm.h`：扩展 `MOTOR_Vars_t`，加入 eSMO handle、反馈模式、角度/速度、误差、启动和接管计数、电压电流缓存等字段，并更新默认初始化。
- `libraries/fcl/source/fcl_cla_code_dm.cla` 与 `sources/fcl_cla_code_dm.cla`：eSMO 模式下禁止 CLA 用 QEP index 提前切 `ENC_CALIBRATION_DONE`，改由 CPU eSMO 接管条件统一控制。
- `sources/dual_axis_servo_drive.c`：加入 eSMO 运行、开环牵引、接管判断、角度同步、速度反馈替换、`esmoStartupLog[]` 和动态削弱启动 Iq 逻辑。
- `sources/dual_axis_servo_drive_sensorless.c`：实现双电机 eSMO adapter，负责 eSMO 输入转换、PLL 运行、角度偏置、接管判断和角度 blend。
- `sources/dual_axis_servo_drive_user.c`：初始化电机参数、eSMO 参数、启动/接管计数和 eSMO handle。
- `sources/fcl_cpu_code_dm.c`：eSMO 模式下 Park 角度使用 `esmoAnglePu`，并保护 QEP wrap 逻辑。

<img width="1706" height="1279" alt="_cgi-bin_mmwebwx-bin_webwxgetmsgimg__ MsgID=791192654624166394 skey=@crypt_e723c0f0_32ac849e5c9778cfac3b8779b34bf9ec mmweb_appid=wx_webfilehelper" src="https://github.com/user-attachments/assets/490ed101-6540-4c41-b28f-a034f5ff2f45" />

## 代码更新日志与硬件实验迭代过程

|     速度 |                          空轴 QEP |                       单接传感器 QEP |                            完整台架 QEP（之前） |                          完整台架 v4.0 eSMO |                            完整台架 v4.0  QEP |
| -----: | ------------------------------: | ------------------------------: | --------------------------------------: | --------------------------------------: | --------------------------------------: |
| 0.05pu | iq = 0.007 ~ 0.016<br>母线 0.017A | iq = 0.010 ~ 0.020<br>母线 0.020A | **iq = 0.093 ~ 0.098**<br>**母线 0.102A** |                                       — |                                       — |
| 0.10pu | iq = 0.012 ~ 0.015<br>母线 0.027A | iq = 0.013 ~ 0.026<br>母线 0.035A | **iq = 0.173 ~ 0.178**<br>**母线 0.332A** | iq = 0.097 ~ 0.110<br>母线 0.165A | iq = 0.104 ~ 0.110<br>母线 0.170A |
| 0.15pu | iq = 0.010 ~ 0.012<br>母线 0.037A | iq = 0.022 ~ 0.025<br>母线 0.051A | **iq = 0.252 ~ 0.257**<br>**母线 0.697A** | iq = 0.136 ~ 0.151<br>母线 0.313A | iq = 0.130 ~ 0.146<br>母线 0.318A |
| 0.18pu | iq = 0.012 ~ 0.017<br>母线 0.044A | iq = 0.016 ~ 0.027<br>母线 0.060A | **iq = 0.295 ~ 0.311**<br>**母线 1.018A** | iq = 0.168 ~ 0.180<br>母线 0.434A | iq = 0.170 ~ 0.181<br>母线 0.482A |
| 0.20pu |                               — |                               — |                                       — | **iq = 0.191 ~ 0.199**<br>**母线 0.524A** |


1. 从单电机无感工程移植 `esmo.c/h`，新增双电机 eSMO adapter，保留 QEP 控制。
2. 修复编译兼容问题：eSMO include path、`POSITION_FEEDBACK_*` 未定义、`MOTOR_Vars_t` 包含顺序、`FCL_Vars_t.positionFeedback` 与 SDK 原始结构体不兼容、CLA 不支持 `<stdlib.h>`。
3. 根据电机参数 txt 修正相电阻、电感和基准量，避免 eSMO/FCL 基准量不匹配导致观测偏差、发热和停转。
4. 先在 QEP 控制下进行 eSMO monitor 测试，验证 `0.05/0.10/0.15/0.20 pu` 稳态速度估计基本可跟随 QEP。
5. 根据 monitor 数据加入 `M1/M2_ESMO_ANGLE_OFFSET_PU=0.40`，补偿 eSMO 角度与 QEP 电角度约 `0.38~0.41 pu` 的固定偏差。
6. 初次 eSMO 接管出现反转、突加速、过流或停转。加入 `esmoStartupLog[]` 后发现 CLA QEP index 会过早切闭环。
7. 增加 CLA QEP index 保护，让 eSMO 模式只能由 CPU 侧 `isSensorlessReadyForClosedLoop()` 切入闭环，解决早切换导致的反转和方向突变。
8. 启动 Iq 从 `0.10 pu` 逐步降到 `0.04 pu`，降低启动扭矩和发热；加入超速削弱逻辑，减少突加速。
9. 将 `M1_STARTUP_IQ_MIN_SCALE` 调到 `0.10`，`M1_STARTUP_OVERSPEED_BAND` 调到 `0.03`，让超速时更早、更强地降低 Iq。
10. 将 `M1_ESMO_FORCE_SPEED` 降到 `0.08` 后一度无法切入闭环，原因是 `rc.SetpointValue≈0.07997` 略低于 `M1_ESMO_TAKEOVER_MIN_SETPOINT=0.08`。把门槛降到 `0.07` 后恢复正常接管。
11. 当前 v4.0：M1 eSMO 单电机可切入 `ENC_CALIBRATION_DONE` 并稳定运行；M2 已具备接口和参数框架，但仍需按 M1 流程单独硬件验证。
