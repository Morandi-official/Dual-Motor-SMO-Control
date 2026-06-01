# Dual-Motor-SMO-Control
本仓库用于备份 `dual_axis_servo_drive_fcl_qep_f2837x` 双电机工程中，相对最初原始工程新增或修改过的代码文件。`v4.0/`、`v5.0/` 等版本目录只放代码文件，不放 CCS 工程元数据 `.project`、编译产物或完整 SDK 工程副本。

## eSMO 切入滑模闭环的具体过程

以 M1 为例，只有同时满足下面两个条件，才表示真正进入 eSMO 无感闭环：

```c
motorVars[0].positionFeedback == POSITION_FEEDBACK_ESMO
motorVars[0].ptrFCL->lsw == ENC_CALIBRATION_DONE
```

若 `M1_POSITION_FEEDBACK=POSITION_FEEDBACK_QEP`，`lsw=ENC_CALIBRATION_DONE` 表示 QEP 闭环；若为 `POSITION_FEEDBACK_ESMO_MONITOR`，控制仍由 QEP 完成，eSMO 只是旁路观测。

当前 M1 切入流程：

1. `ENC_ALIGNMENT`：对齐阶段，`IdRef_start=M1_STARTUP_ID_REF`，Iq 为 0。该阶段只建立一个确定的初始电角度，不使用 eSMO 闭环控制。
2. `ENC_WAIT_FOR_INDEX`：eSMO 开环牵引阶段。eSMO 模式下禁止 CLA 因 QEP index 自动切闭环，CPU 侧继续用开环斜坡角 `rg.Out` 作为 FCL Park 角度，同时 eSMO 在旁路运行并积累反电动势、PLL 角度和速度估计。开环目标由 `M1_ESMO_FORCE_SPEED` 给定，最少强拖时间由 `M1_ESMO_FORCE_RUN_SEC` 给定。
3. 启动 Iq 控制：`getSensorlessStartupIqRef()` 只在 `ENC_WAIT_FOR_INDEX` 阶段直接给 FCL 的 `pi_iq.ref`。当前逻辑先按 `M1_STARTUP_IQ_MIN_SCALE` 从较低 Iq 软启动，随后随 `esmoForceRunCntr` 逐步爬升到 `M1_STARTUP_IQ_REF`；若 `esmoSpeedPu` 已经超过当前 `rc.SetpointValue`，再按 `M1_STARTUP_OVERSPEED_BAND` 做超速削弱，避免开环牵引阶段过冲。
4. 接管判断：强拖时间满足后，要求 `rc.SetpointValue>=M1_ESMO_TAKEOVER_MIN_SETPOINT`、`abs(esmoSpeedPu)>=M1_ESMO_TAKEOVER_MIN_SPEED`、`Eq_mag>=M1_ESMO_TAKEOVER_MIN_BEMF`，且 `rc.SetpointValue` 与 `esmoSpeedPu` 方向一致。满足后调用 `syncSensorlessEstimatorAngle()`，把 eSMO 内部角度同步到当前开环斜坡角附近，并置 `lsw=ENC_CALIBRATION_DONE`。
5. 接管瞬间：`lsw` 置为 `ENC_CALIBRATION_DONE` 之后，FCL Park 角度来源切换为 `esmoAnglePu`，速度反馈切换为 `esmoSpeedPu`。同时调用 `seedSensorlessSpeedController()`，用切换前的启动 Iq 给速度 PI 的积分项做初值种入，避免 `pi_iq.ref` 在启动 Iq 与速度 PI 输出之间突跳。
6. 接管过渡期：`isSensorlessTakeoverActive()` 并不是“是否已经进入 eSMO 闭环”的唯一判据，它表示 `lsw=ENC_CALIBRATION_DONE` 之后的一段角度混合期，长度由 `M1_ESMO_TAKEOVER_SEC` 决定。在这段时间里，`blendSensorlessAngle()` 按 `esmoTakeoverCntr/esmoTakeoverCntMax` 形成 `alpha`，把控制角度从开环斜坡角逐步混合到 eSMO 观测角：

```c
angleErrPu = wrapPuHalf(estimatorAnglePu - openLoopAnglePu);
esmoAnglePu = normalizePu(openLoopAnglePu + alpha * angleErrPu);
```

这里的“角度混合”意思是：刚切入时 `alpha` 接近 0，控制角度仍接近开环斜坡角；随后 `alpha` 逐渐增大，控制角度平滑靠近 eSMO 角度；最后 `alpha=1`，控制角度完全由 eSMO 角度决定。这样做是为了避免 Park 角度从开环角突然跳到 eSMO 角度，引发 q 轴电流、转矩和转速突变。

早期版本中，接管过渡期不仅用于角度混合，还在 `pi_iq.ref` 里继续使用 `getSensorlessStartupIqRef()`。因此即使 `lsw` 已经变成 `ENC_CALIBRATION_DONE`，约 `M1_ESMO_TAKEOVER_SEC` 秒内的 Iq 仍主要来自启动转矩逻辑，而不是正常速度 PI。0.2pu 实验中出现的多次 `piIqRef_q15=3276`，就是这一逻辑导致的 `IqRef=0.10pu` 启动转矩反复参与，而不是协同控制器或普通速度 PI 本身过强。当前版本已将这两件事拆开：接管过渡期只继续做角度混合，Iq 则在切入后交给 eSMO 专用速度 PI 调节。

7. `ENC_CALIBRATION_DONE` 稳态：eSMO 提供电角度和速度，FCL Park 角度使用 `esmoAnglePu`，速度闭环输出 `pid_spd.term.Out` 作为 Iq 主指令。eSMO 模式下速度 PI 使用较保守的 `M*_ESMO_SPEED_PID_KP/KI`，目的是降低 `esmoSpeedPu` 波动被速度环放大成 Iq 波动的程度；QEP 模式仍保留原速度 PI 参数。

最近一次 0.2pu 启动日志显示：切换点约在 `esmoStartupLog[49]`，`rcSetpoint≈0.0726 pu`、`esmoSpeed≈0.0877 pu`、`Eq_mag≈0.0516 pu`；启动峰值 `esmoStartupPeakSpeedPu≈0.223 pu`、`esmoStartupPeakIqRef=0.10 pu`、`esmoStartupPeakIqFbk≈0.194 pu`。该数据说明软启动已经降低了电源启动电流，但切入后的 Iq 过渡仍需要从“启动转矩保持”改为“速度 PI 接管并用积分初值平滑衔接”。

## 代码文件修改说明

### v4.0（2026-05-27 提交）

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

### v5.0（2026-05-30 提交）

- `include/dual_axis_servo_drive_user.h`：将默认反馈源从单电机/调试阶段切换为双电机 eSMO 实控，`M1_POSITION_FEEDBACK` 与 `M2_POSITION_FEEDBACK` 均设为 `POSITION_FEEDBACK_ESMO`。
- `include/dual_axis_servo_drive_user.h`：根据完整实验平台负载条件，将 M1 启动 Iq 从空轴可用的 `M1_STARTUP_IQ_REF=0.04`、`M1_STARTUP_IQ_MIN_SCALE=0.10` 调整为 `0.10`、`0.20`。
- `include/dual_axis_servo_drive_user.h`：将 M2 启动 Iq 同步为 `M2_STARTUP_IQ_REF=0.10`、`M2_STARTUP_IQ_MIN_SCALE=0.20`，使两套同型号电机和机械台架使用一致的 eSMO 启动扭矩策略。
- 其余代码文件随 v5.0 一并备份，内容保持当前工程状态，便于在真实 CCS 项目中按版本整体复刻。

## 代码更新日志与硬件实验迭代过程

### v4.0（2026-05-27 提交）

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

### v5.0（2026-05-30 提交）

1. 在被控电机空轴条件下，`M1_STARTUP_IQ_REF=0.04`、`M1_STARTUP_IQ_MIN_SCALE=0.10` 可以正常启动并切入 eSMO 闭环。
2. 将被控电机接入完整实验平台后，机械系统变为“被控电机 + 转矩传感器 + 另一台同型号电机机械连接”，启动负载和静摩擦明显高于空轴条件。
3. 在完整台架负载下，空轴启动 Iq 参数不足以可靠启动和切入滑模闭环；经硬件测试，`STARTUP_IQ_REF=0.10`、`STARTUP_IQ_MIN_SCALE=0.20` 可以正常启动和进入 `ENC_CALIBRATION_DONE`。
4. 基于两套平台和所有电机均为同型号的前提，将 M1/M2 启动 Iq 参数统一为 `0.10/0.20`。
5. 单电机 eSMO 移植功能已初步通过后，将默认反馈源切换为双电机 `POSITION_FEEDBACK_ESMO`，用于下一阶段双电机滑模控制测试。
6. 本次没有改变 eSMO 接管判据、角度偏置、强拖速度、强拖时间和 CLA/QEP index 保护逻辑，因此“eSMO 切入滑模闭环的具体过程”主体保持 v4.0 描述不变。

<img width="1706" height="1279" alt="_cgi-bin_mmwebwx-bin_webwxgetmsgimg__ MsgID=791192654624166394 skey=@crypt_e723c0f0_32ac849e5c9778cfac3b8779b34bf9ec mmweb_appid=wx_webfilehelper" src="https://github.com/user-attachments/assets/490ed101-6540-4c41-b28f-a034f5ff2f45" />
