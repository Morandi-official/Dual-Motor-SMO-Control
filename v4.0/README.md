# v4.0 代码备份说明

本目录只备份 `dual_axis_servo_drive_fcl_qep_f2837x` 相对最初原始工程被修改或新增的代码文件，不包含 CCS 工程元数据 `.project`，也不包含编译产物。

## 代码文件修改说明

- `include/dual_axis_servo_drive.h`
  - 引入 `dual_axis_servo_drive_sensorless.h`，使主控代码可以访问 eSMO 初始化、运行、接管判断和角度同步接口。
  - 保留原 QEP 代码路径，未删除原有编码器闭环能力。

- `include/dual_axis_servo_drive_sensorless.h`
  - 新增双电机 eSMO 适配层头文件。
  - 使用 `struct _MOTOR_Vars_t_` 前向声明，避免和 `MOTOR_Vars_t` 的包含顺序形成循环依赖。
  - 提供 `initSensorlessEstimator()`、`resetSensorlessEstimator()`、`runSensorlessEstimator()`、`isSensorlessControl()`、`isSensorlessReadyForClosedLoop()`、`forceSensorlessAngle()`、`blendSensorlessAngle()`、`syncSensorlessEstimatorAngle()` 等接口。

- `include/dual_axis_servo_drive_settings.h`
  - 增加 `POSITION_FEEDBACK_QEP`、`POSITION_FEEDBACK_ESMO_MONITOR`、`POSITION_FEEDBACK_ESMO` 的兼容定义。
  - 将反馈模式选择最终转移到 `dual_axis_servo_drive_user.h`，避免 CLA 编译器包含 driverlib-heavy 的 settings 头文件。

- `include/dual_axis_servo_drive_user.h`
  - 增加 CLA 安全的反馈源宏：`POSITION_FEEDBACK_QEP`、`POSITION_FEEDBACK_ESMO_MONITOR`、`POSITION_FEEDBACK_ESMO`。
  - 增加 `M1_POSITION_FEEDBACK`、`M2_POSITION_FEEDBACK`，用于运行前选择 QEP、eSMO 旁路观测或 eSMO 接管。
  - 根据电机参数 txt 修正双电机参数：相电阻由线电阻 `0.59 ohm` 折算为 `0.295 ohm`，`Ld=0.0010233 H`，`Lq=0.0010302 H`，平均 `Ls=0.00102675 H`，`BASE_CURRENT=13.5 A`，`BASE_FLUX=0.0114`，`BASE_FREQ=266.6667 Hz`，`BASE_VOLTAGE=13.86 V`。
  - 增加 M1/M2 eSMO 参数：`Kslide`、PLL Kp 范围、BEMF 阈值、速度滤波、角度补偿、开环牵引速度、接管时间和接管门槛。
  - 当前 M1 经过硬件调试后的关键参数为：`M1_ESMO_FORCE_SPEED=0.08`，`M1_ESMO_TAKEOVER_MIN_SETPOINT=0.07`，`M1_STARTUP_IQ_REF=0.04`，`M1_STARTUP_IQ_MIN_SCALE=0.10`，`M1_STARTUP_OVERSPEED_BAND=0.03`。
  - M2 暂保守保留为待独立验证参数：`M2_ESMO_FORCE_SPEED=0.10`，`M2_STARTUP_IQ_REF=0.05`，`M2_STARTUP_IQ_MIN_SCALE=0.20`，`M2_STARTUP_OVERSPEED_BAND=0.06`。

- `include/esmo.h`
  - 从单电机无感工程移植 eSMO 对象、参数和接口定义。
  - 调整 include 依赖，避免引用 SDK 中不适配当前工程 include path 的 `libraries/math/include/math.h`。
  - 增加角度、电角速度、PLL、反电势、滑模增益相关访问接口，供双电机适配层调用。

- `include/fcl_enum.h`
  - 增加位置/速度反馈源枚举兼容宏，保证当真实工程仍链接 SDK 原始头文件时，`POSITION_FEEDBACK_*` 不会未定义。

- `libraries/fcl/include/fcl_cpu_cla_dm.h`
  - 扩展 `MOTOR_Vars_t`，加入 eSMO 运行所需字段：反馈模式、eSMO handle、角度/速度、角度误差、QEP 对比角、启动计数、接管计数、电流电压输入缓存等。
  - 修改 `MOTOR1_DEFAULTS`、`MOTOR2_DEFAULTS`，初始化新增 eSMO 字段并绑定 `M1/M2_POSITION_FEEDBACK`。
  - 引入 `dual_axis_servo_drive_settings.h` 以让 CPU 侧默认参数看到反馈选择和 eSMO 参数。

- `libraries/fcl/source/fcl_cla_code_dm.cla`
  - 增加 CLA 侧 QEP index 自动切换保护。
  - 在 `POSITION_FEEDBACK_ESMO` 模式下禁止 CLA 因 QEP index 事件直接把 `lsw` 切到 `ENC_CALIBRATION_DONE`，避免 eSMO 接管门槛被绕过。
  - 去除了对 `fclVars[].positionFeedback` 结构体字段的依赖，改为编译期宏判断，兼容 SDK 原始 `FCL_Vars_t`。

- `sources/dual_axis_servo_drive.c`
  - 在主控制流程中加入 eSMO 初始化、运行、开环强拖、接管判断、角度同步和速度反馈替换逻辑。
  - 保留 QEP 和 eSMO monitor 路径：QEP 模式仍使用编码器控制；monitor 模式 QEP 控制、eSMO 旁路观测；eSMO 模式使用 eSMO 角度和速度闭环。
  - 增加 `esmoStartupLog[]` 瞬态锁存变量，用于记录 `lsw`、开环角、eSMO 速度、QEP 角度、角度误差、BEMF、PLL 输出、Iq ref/fbk 等启动过程数据。
  - 增加 `getSensorlessStartupIqRef()`，在 eSMO 开环强拖和接管过渡阶段根据超速量动态削弱 Iq，减少启动突加速。
  - 修改 M1/M2 初始化启动电流，改由 `M1/M2_STARTUP_ID_REF`、`M1/M2_STARTUP_IQ_REF` 参数控制。
  - 在 eSMO 接管时调用 `syncSensorlessEstimatorAngle()`，用开环角同步 eSMO 内部角度，减少切换冲击。

- `sources/dual_axis_servo_drive_sensorless.c`
  - 新增双电机 eSMO 适配层实现。
  - 将三相电压、电流转换为 eSMO 输入，运行滑模观测器和 PLL，输出 `esmoAnglePu`、`esmoSpeedPu`、`esmoSpeedHz`。
  - 实现 eSMO 接管条件：开环设定速度、估算速度、BEMF 幅值、速度方向一致性均满足后才允许 `lsw=ENC_CALIBRATION_DONE`。
  - 实现角度偏置补偿、角度强制、角度同步和接管期间角度 blend。

- `sources/dual_axis_servo_drive_user.c`
  - 初始化电机参数、eSMO 参数、启动/接管计数和 eSMO handle。
  - 初始化 M1/M2 `positionFeedback`，使运行时可根据宏选择 QEP、eSMO monitor 或 eSMO 控制。
  - 为 eSMO 建立 PWM 标幺比例、角度延迟补偿、启动时间、接管时间等参数。

- `sources/esmo.c`
  - 从单电机无感工程移植 eSMO 算法实现。
  - 包含滑模电流估算、反电势估算、Kslide 更新、PLL 角度和速度估算等逻辑。
  - 适配当前双电机工程的类型、math helper 和 include path。

- `sources/fcl_cla_code_dm.cla`
  - 同 `libraries/fcl/source/fcl_cla_code_dm.cla`，用于工程本地 linked source 场景。
  - 保证真实 CCS 工程无论链接工程内副本还是 SDK 路径副本，都有相同的 QEP index 保护逻辑。

- `sources/fcl_cpu_code_dm.c`
  - 修改 FCL CPU 电流环相关路径，使 `POSITION_FEEDBACK_ESMO` 模式下 Park 角度使用 eSMO 角度。
  - 对 QEP wrap 相关逻辑增加 eSMO 模式保护，避免无感控制时仍依赖 QEP wrap 更新。

## 代码更新日志与硬件实验迭代过程

1. 初始移植阶段
   - 从 `universal_motorcontrol_lab_f28002x` 提取 `esmo.c`、`esmo.h` 的滑模观测器功能。
   - 在双电机 FCL/QEP 工程中新增 `dual_axis_servo_drive_sensorless.c/h`，把 eSMO 封装成可按电机实例调用的适配层。
   - 保留原 QEP 控制路径，新增三种反馈模式：QEP、eSMO monitor、eSMO control。

2. 编译兼容阶段
   - 修复 `esmo.h` 引用 `libraries/math/include/math.h` 导致真实工程找不到头文件的问题。
   - 增加 `POSITION_FEEDBACK_*` fallback，解决真实 CCS 工程链接 SDK 原始头文件时宏未定义的问题。
   - 将 `dual_axis_servo_drive_sensorless.h` 改成前向声明 `struct _MOTOR_Vars_t_`，解决 `MOTOR_Vars_t undefined` 的包含顺序问题。
   - 撤回 `FCL_Vars_t.positionFeedback` 字段方案，改用 CLA 编译期宏判断，解决 SDK 原始 FCL 结构体无该字段的问题。
   - 避免 CLA include `dual_axis_servo_drive_settings.h`，解决 CLA 编译器不支持 `<stdlib.h>` 的问题。

3. 电机参数修正阶段
   - 根据电机参数 txt 修正电阻、电感、基准电流、基准电压、基准磁链和基准频率。
   - 明确 `RS_LINE=0.59 ohm` 为线电阻，FCL/eSMO 使用相电阻 `RS=0.295 ohm`。
   - 使用 d/q 轴电感 `Ld=0.0010233 H`、`Lq=0.0010302 H`，并保留平均 `Ls=0.00102675 H`。

4. eSMO monitor 旁路观测阶段
   - 在 QEP 控制下旁路运行 eSMO，对比 `speed.Speed`、`esmoSpeedPu`、`posElecTheta`、`esmoAnglePu`、`Eq_mag`、`thetaErr`、`pll_Out`。
   - 测试 `0.05/0.10/0.15/0.20 pu` 速度点，确认 eSMO 速度估计在稳态下可跟随 QEP 速度。
   - 发现 eSMO 原始角度与 QEP 电角度存在约 `0.38~0.41 pu` 固定偏差，因此加入 `M1/M2_ESMO_ANGLE_OFFSET_PU=0.40`。

5. eSMO 初次接管问题
   - 初次切到 `POSITION_FEEDBACK_ESMO` 后出现启动反转、突然加速、过流保护或大幅速度摆动。
   - 加入 `esmoStartupLog[]`，锁存 `lsw` 切换前后的 `rcSetpoint`、开环角、估算速度、eSMO 角、QEP 角、BEMF、PLL 输出和 Iq ref/fbk。
   - 通过日志发现 `lsw` 在很低速度时被 QEP index 事件提前切到 `ENC_CALIBRATION_DONE`。

6. CLA QEP index 保护阶段
   - 修改 CLA QEP 任务，在 `POSITION_FEEDBACK_ESMO` 下不再由 QEP index 自动切换 `lsw=ENC_CALIBRATION_DONE`。
   - 改为 CPU 侧 eSMO 接管判断统一控制切换，接管条件包含开环速度、估算速度、BEMF 和方向一致性。
   - 解决早切换导致的反转和明显方向突变问题。

7. 接管门槛和启动过程调试
   - 起初 `M1_ESMO_FORCE_SPEED=0.10`、`M1_STARTUP_IQ_REF=0.10`，启动扭矩过大，出现较明显突加速。
   - 将 M1 启动 Iq 从 `0.10 pu` 降到 `0.05 pu`，启动突加速减弱。
   - 继续降到 `M1_STARTUP_IQ_REF=0.04`，并加入超速削弱逻辑：当 eSMO 估算速度超过当前开环 setpoint 后自动降低 Iq。
   - 将 `M1_STARTUP_IQ_MIN_SCALE` 调为 `0.10`，最低启动/接管 Iq 可降至 `0.004 pu`。
   - 将 `M1_STARTUP_OVERSPEED_BAND` 从 `0.06` 调为 `0.03`，使超速时更早削弱 Iq。

8. FORCE_SPEED 与接管门槛配合调试
   - 将 `M1_ESMO_FORCE_SPEED` 从 `0.10` 降到 `0.08` 后，发现卡在 `ENC_WAIT_FOR_INDEX`，无法进入 `ENC_CALIBRATION_DONE`。
   - 日志显示 `rc.SetpointValue=0.079971821`，略低于原 `M1_ESMO_TAKEOVER_MIN_SETPOINT=0.08`，因此接管条件被硬门槛挡住。
   - 将 `M1_ESMO_TAKEOVER_MIN_SETPOINT` 从 `0.08` 降到 `0.07`，保留 `M1_ESMO_FORCE_SPEED=0.08`。
   - 之后测试显示可正常切入 `ENC_CALIBRATION_DONE`，切换点约 `rcSetpoint=0.0739 pu`、`speedFbk=0.0862 pu`、`Eq_mag=0.0593 pu`。

9. 当前 v4.0 状态
   - M1 eSMO 单电机接管测试已能进入 `ENC_CALIBRATION_DONE`，稳态速度约 `0.1006 pu`，`esmoSpeedPu` 约 `0.0972 pu`，`tripFlagDMC=0`。
   - 当前版本仍建议保留编码器用于观察和安全验证；物理断开编码器或双电机同时 eSMO 接管应作为后续测试阶段。
   - M2 已具备同样接口和参数框架，但 M2 的启动参数仍需按 M1 的流程单独硬件验证后再收敛。
