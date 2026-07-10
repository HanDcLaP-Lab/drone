# 无名飞控可借鉴逻辑与算法梳理

本文档面向当前 `drone/` 项目，阅读对象是仓库根目录下的 `../无名飞控/`。两者核心板、外设和任务目标不同，因此这里不建议直接移植整套工程；更有价值的是抽取其中已经被飞控项目验证过的控制链路、传感器有效性判断、故障降级和状态机写法。

## 阅读范围

重点阅读了无名飞控这些模块：

- 调度入口：`../无名飞控/maplepilot/user/main_cm4.c`、`../无名飞控/maplepilot/user/cm4_isr.c`、`../无名飞控/fc_driver/driver/schedule.c`
- 传感器与融合：`../无名飞控/fc_driver/algorithm/sensor.c`、`sins.c`、`maple_ahrs.c`、`quaternion.c`、`gps_ekf.c`、`wp_math.c`
- 控制算法：`../无名飞控/fc_driver/algorithm/pid.c`、`filter.c`
- 飞控链路：`../无名飞控/fc_driver/control/flymaple_ctrl.c`、`attitude_ctrl.c`、`altitude_ctrl.c`、`position_ctrl.c`、`control_output_mix.c`
- 外设观测：`../无名飞控/fc_driver/driver/drv_tofsense.c`、`drv_opticalflow.c`

对照阅读了当前 `drone/` 的：

- `project/code/imu.c`、`tof.c`、`fly_ctrl.c`、`pid.c`、`filters.c`
- `project/code/image.c`、`image_ctrl.c`、`data_complex.c`
- `project/user/main_cm7_0.c`、`main_cm7_1.c`、`cm7_0_isr.c`、`cm7_1_isr.c`

## 总体结论

无名飞控最值得借鉴的不是某一个 PID 参数，而是这几个系统性习惯：

1. 每条控制链路都带真实时间、任务耗时、传感器有效性和超时判断。
2. 姿态/高度/位置控制都采用级联结构，并在模式切换、失效和落地时重捕获目标或复位积分。
3. 对 ToF、气压计、GPS、光流、SLAM 等观测源先做质量门控，再融合进 SINS/Kalman/互补滤波。
4. 对飞控安全事件有专门的“降级动作”：落地检测、Yaw 故障检测、低电压计数、指南针卡死检测、主观测源失效重置等。
5. 所有“可调参数”和“控制器状态”被集中管理，便于后续现场调参和记录。

当前 `drone/` 已经具备部分相同方向的改进，例如：

- TOF 高度 PID 已在 `tof_update()` 中按数据就绪和实测 `dt` 更新，见 `drone/project/code/tof.c:180`、`drone/project/code/tof.c:195`、`drone/project/code/tof.c:202`。
- 高度油门已做倾角补偿，见 `drone/project/code/fly_ctrl.c:170` 到 `drone/project/code/fly_ctrl.c:179`。
- 姿态控制为角度环到角速度环的级联，见 `drone/project/code/fly_ctrl.c:182` 到 `drone/project/code/fly_ctrl.c:210`。
- Yaw 已改为使用机体系角速度换算后的 yaw rate 积分，避免 roll/pitch 加速度修正耦合进 yaw，见 `drone/project/code/imu.c:457` 到 `drone/project/code/imu.c:466`。
- 视觉位置环使用曝光瞬间 yaw 快照计算地球系误差，再用实时 yaw 转回机体系，见 `drone/project/code/image_ctrl.c:87` 到 `drone/project/code/image_ctrl.c:117`。

剩余差距主要在“质量门控、故障降级、延迟补偿、积分复位和任务监控”。

## 优先级建议

| 优先级 | 可借鉴点 | 迁移难度 | 对当前 drone 的价值 |
|---|---|---:|---|
| P0 | 控制循环真实 dt 防护、任务耗时记录、PID 积分复位策略 | 低 | 降低偶发超时/丢帧导致的控制尖峰 |
| P0 | TOF 有效性、标准差、突变和失效降级 | 中 | 对“飞行中偶尔掉高/突然加速倾倒”很关键 |
| P0 | Yaw/姿态故障检测，异常时重置目标和积分 | 中 | 防止某轴输出很大但机体响应异常时继续积分 |
| P1 | 高度 SINS/Kalman：ToF + 竖直加速度 + 气压备份 | 中高 | 比 TOF-only 更稳，但需要谨慎调试 |
| P1 | 目标重捕获/停止点逻辑：速度衰减后再锁定悬停点 | 中 | 对视觉目标短丢、换向和滑行更柔和 |
| P1 | 光流/视觉观测延迟补偿和历史状态对齐 | 中高 | 改善视觉闭环时序误差 |
| P2 | 电机混控矩阵和统一抗饱和 | 中 | 当前 X4 手写混控够用，长期可提升可维护性 |
| P2 | IMU 温控稳定判据 | 硬件相关 | 若没有加热硬件，不直接适用，但温漂监测可借鉴 |
| P3 | GPS EKF、磁偏角、航点数学 | 高/暂不适用 | 当前室内视觉任务收益较低，可作为长期储备 |

## 1. 调度与真实时间

无名飞控把主控制任务拆成两个 200Hz PIT：`maple_duty1_200hz()` 负责传感器、融合、导航、遥控、控制和反馈，`maple_duty2_200hz()` 负责数传、校准、参数服务等后台任务。入口见 `../无名飞控/maplepilot/user/main_cm4.c:119` 到 `../无名飞控/maplepilot/user/main_cm4.c:120`，ISR 调用见 `../无名飞控/maplepilot/user/cm4_isr.c:40` 到 `../无名飞控/maplepilot/user/cm4_isr.c:52`。`maple_duty1_200hz()` 内部顺序是 `sensor_raw_update()`、`sensor_fusion_update()`、`flymaple_nav_update()`、`flymaple_ctrl()`，见 `../无名飞控/maplepilot/user/main_cm4.c:131` 到 `../无名飞控/maplepilot/user/main_cm4.c:150`。

它还在 `schedule.c` 里用 SysTick 维护微秒时间，`micros()` 通过 2ms tick 加当前 SysTick 计数得到亚毫秒时间，见 `../无名飞控/fc_driver/driver/schedule.c:73` 到 `../无名飞控/fc_driver/driver/schedule.c:75`；`get_systime()` 给每个任务算 `period` 和 `period_int`，见 `../无名飞控/fc_driver/driver/schedule.c:117` 到 `../无名飞控/fc_driver/driver/schedule.c:127`。

当前 `drone/` 已经有 CM7_0 的 1ms `dataC.pit0_cnt`，TOF 和飞控 ISR 分离，见 `drone/project/user/cm7_0_isr.c:50` 到 `drone/project/user/cm7_0_isr.c:68`；CM7_1 也有本地 10ms 计时，见 `drone/project/user/cm7_1_isr.c:64` 到 `drone/project/user/cm7_1_isr.c:67`。可以借鉴的是：

- 给 `Flight_Control_Loop()`、`IMU_Update_Loop()`、`Flight_Hover_Control_Task()` 记录最大耗时和实际周期，用于判断现场偶发卡顿。
- 所有控制器内部都对 `dt` 做上下限保护，而不是默认 ISR 永远准时。
- 当视觉/TOF/IMU 更新间隔超限时，重捕获目标或降级，而不是继续使用旧目标。

## 2. PID、滤波和 NaN 防护

无名飞控的 PID 比当前 `drone/project/code/pid.c` 更完整。当前 `drone` 的 `PID_Calculate()` 有积分限幅、输出限幅和一阶微分滤波，见 `drone/project/code/pid.c:44` 到 `drone/project/code/pid.c:87`。无名飞控在此基础上增加了：

- `pid_ctrl_general()` 对 `dt` 做 0.95 到 1.05 倍周期的保护，异常时退回名义周期，见 `../无名飞控/fc_driver/algorithm/pid.c:155` 到 `../无名飞控/fc_driver/algorithm/pid.c:161`。
- 姿态角速度环 `pid_ctrl_rpy_gyro()` 支持不同微分模式和低通模式，见 `../无名飞控/fc_driver/algorithm/pid.c:200` 到 `../无名飞控/fc_driver/algorithm/pid.c:239`。
- `pid_ctrl_yaw()` 单独处理 yaw 环绕，见 `../无名飞控/fc_driver/algorithm/pid.c:293` 到 `../无名飞控/fc_driver/algorithm/pid.c:298`。
- `takeoff_ctrl_reset()` 和 `optical_ctrl_reset()` 把起飞/降落/光流失效时应清除的控制器集中复位，见 `../无名飞控/fc_driver/algorithm/pid.c:512`、`../无名飞控/fc_driver/algorithm/pid.c:542`。

滤波方面，无名飞控用二阶 Butterworth，`set_cutoff_frequency()` 计算系数，`butterworth()` 运行时检查输入/输出 NaN 并重置，见 `../无名飞控/fc_driver/algorithm/filter.c:45` 到 `../无名飞控/fc_driver/algorithm/filter.c:84`。当前 `drone` 有一维 Kalman 和 Notch，见 `drone/project/code/filters.c:8` 到 `drone/project/code/filters.c:54`，但缺少统一的 NaN/Inf 防护。

建议优先迁移：

- 给 `PID_Calculate()` 增加 `dt` 合法区间保护。
- 给姿态/高度/视觉 PID 增加“误差过大时积分分离”或“失效时统一 reset”策略。
- 在 `filters.c` 的 Kalman/Notch 输出处加入 NaN/Inf 复位保护。

## 3. IMU 与姿态融合

无名飞控 `sensor_raw_update()` 的传感器预处理链路很完整：读取 IMU、气压计、磁力计，按量程转换，做校准、低通、单位转换，再生成观测角和备份历史。关键代码见 `../无名飞控/fc_driver/algorithm/sensor.c:201` 到 `../无名飞控/fc_driver/algorithm/sensor.c:302`。它还检测指南针原始数据是否长时间不变，见 `../无名飞控/fc_driver/algorithm/sensor.c:173` 到 `../无名飞控/fc_driver/algorithm/sensor.c:196`。

姿态融合方面，无名飞控保留了多套实现：

- `maple_ahrs.c` 有四元数 EKF 初始化和更新，见 `../无名飞控/fc_driver/algorithm/maple_ahrs.c:117` 到 `../无名飞控/fc_driver/algorithm/maple_ahrs.c:209`。
- Madgwick 会根据陀螺模长和加速度模长调整修正强度，见 `../无名飞控/fc_driver/algorithm/maple_ahrs.c:241` 到 `../无名飞控/fc_driver/algorithm/maple_ahrs.c:330`。
- Mahony 会按加速度模长衰减反馈，并在角速度较小时才积分误差，见 `../无名飞控/fc_driver/algorithm/maple_ahrs.c:379` 到 `../无名飞控/fc_driver/algorithm/maple_ahrs.c:418`。
- 当前主链路使用 FusionAhrs，初始化时设置外部 heading / 无磁模式 / rejection 参数，见 `../无名飞控/fc_driver/algorithm/sensor.c:429` 到 `../无名飞控/fc_driver/algorithm/sensor.c:462`；更新时可选外部航向或无磁力计更新，见 `../无名飞控/fc_driver/algorithm/sensor.c:363` 到 `../无名飞控/fc_driver/algorithm/sensor.c:387`。

当前 `drone` 的 `IMU_Update_Loop()` 已有原始更新率计数、Notch、Kalman、静止校准、轴映射、Mahony 和 yaw rate 积分，见 `drone/project/code/imu.c:295` 到 `drone/project/code/imu.c:468`。更值得借鉴的是：

- 加速度模长门控：在大机动或振动时减弱 roll/pitch 的加速度修正，避免姿态被非重力加速度拉偏。
- IMU 温度稳定判据或温漂监测：无名飞控有 IMU 温控目标 50 摄氏度和稳定判断，见 `../无名飞控/fc_driver/control/attitude_ctrl.c:381` 到 `../无名飞控/fc_driver/control/attitude_ctrl.c:401`。如果当前硬件没有加热器，也可以只记录温度和零偏漂移。
- 姿态融合初始化应明确依赖“静止且高度/温度稳定”条件。无名飞控在温度稳定后初始化四元数，见 `../无名飞控/fc_driver/algorithm/sensor.c:443` 到 `../无名飞控/fc_driver/algorithm/sensor.c:453`。

## 4. 高度估计与 ToF 降级

当前 `drone` 的 TOF 链路已经比早期安全：VL53L8CX 多区域测距先取有效点的截尾均值，见 `drone/project/code/tof.c:30` 到 `drone/project/code/tof.c:72`；高度按 roll/pitch 倾角投影，见 `drone/project/code/tof.c:79` 到 `drone/project/code/tof.c:90`；高度和速度一阶低通后进入高度位置/速度 PID，见 `drone/project/code/tof.c:94` 到 `drone/project/code/tof.c:117`；`tof_update()` 用 `dataC.pit0_cnt` 算实测 `dt`，见 `drone/project/code/tof.c:180` 到 `drone/project/code/tof.c:220`。

无名飞控值得补充的是“高度不是只有一个观测源”：

- `height_sensor_select()` 优先使用 ToF，有效时把 ToF 距离作为高度观测；无效时回到气压高度，见 `../无名飞控/fc_driver/algorithm/sensor.c:468` 到 `../无名飞控/fc_driver/algorithm/sensor.c:506`。
- `flymaple_nav_update()` 把惯性加速度、ToF/气压高度和 Kalman/SINS 连起来，见 `../无名飞控/fc_driver/algorithm/sensor.c:511` 到 `../无名飞控/fc_driver/algorithm/sensor.c:534`。
- `altitude_kalman_filter()` 用竖直加速度预测位置/速度，并在观测更新时修正位置、速度和加速度偏置，见 `../无名飞控/fc_driver/algorithm/sins.c:97` 到 `../无名飞控/fc_driver/algorithm/sins.c:195`。
- `strapdown_ins_height()` 提供三阶互补高度融合，见 `../无名飞控/fc_driver/algorithm/sins.c:402` 到 `../无名飞控/fc_driver/algorithm/sins.c:426`。
- ToF 驱动不只判断距离，还判断信号强度、距离范围、更新周期和高度标准差，见 `../无名飞控/fc_driver/driver/drv_tofsense.c:59` 到 `../无名飞控/fc_driver/driver/drv_tofsense.c:89`，以及 TFmini/SMD15 的有效性判断 `../无名飞控/fc_driver/driver/drv_tofsense.c:248` 到 `../无名飞控/fc_driver/driver/drv_tofsense.c:262`、`../无名飞控/fc_driver/driver/drv_tofsense.c:439` 到 `../无名飞控/fc_driver/driver/drv_tofsense.c:453`。

建议给 `drone` 增加的不是完整 GPS/SINS，而是一个小型高度观测管理层：

- 记录最近 N 帧 TOF 的均值、标准差、突变幅度、连续无效帧数。
- TOF 突然无效时，不要立即让 `imu_data.z/vz` 跳变；短时保持预测，超时后降级停车或降落。
- 若有气压计可用，长期用气压兜底，近地用 ToF 校正。
- 高度 PID 进入失效/低置信度时清积分并限制油门变化率。

## 5. 高度控制、起飞/落地和油门补偿

无名飞控 `flight_altitude_control()` 是高度位置环、速度环、加速度/油门输出的级联控制，见 `../无名飞控/fc_driver/control/altitude_ctrl.c:48`。它在油门居中、模式切换和调度异常时重捕获目标高度，并限制上下加速度输出。`throttle_angle_compensate()` 用 roll/pitch 余弦补偿倾角导致的垂直升力损失，见 `../无名飞控/fc_driver/control/altitude_ctrl.c:246`。当前 `drone` 已有类似倾角补偿，见 `drone/project/code/fly_ctrl.c:170` 到 `drone/project/code/fly_ctrl.c:179`。

更值得借鉴的是落地检测：`landon_earth_check()` 组合低油门、低角速度、低竖直速度和计数器判断落地，见 `../无名飞控/fc_driver/control/altitude_ctrl.c:201`。当前 `drone` 的 `Flight_State_Update()` 主要依靠目标状态和高度阈值切换 normal / pre_landing / landing，见 `drone/project/code/fly_ctrl.c:116` 到 `drone/project/code/fly_ctrl.c:161`。

建议：

- 在降落和异常锁定时加入“低油门 + 低姿态角速度 + 低高度/低速度”的组合判据。
- 起飞 ramp 期间限制姿态角输出和高度积分，避免刚解锁时积分残留。
- 当高度观测不可信时，冻结目标高度并清高度速度环积分。

## 6. 姿态控制、Yaw 故障和目标保持

无名飞控主控制入口 `flymaple_ctrl()` 的链路是：上层模式控制、落地检测、锁定判断、姿态控制、输出混控，见 `../无名飞控/fc_driver/control/flymaple_ctrl.c:236` 到 `../无名飞控/fc_driver/control/flymaple_ctrl.c:244`。`flymaple_output()` 根据油门状态决定是否输出姿态控制、是否进入 idle ramp、是否调用 `takeoff_ctrl_reset()`，见 `../无名飞控/fc_driver/control/flymaple_ctrl.c:149` 到 `../无名飞控/fc_driver/control/flymaple_ctrl.c:193`。

当前 `drone` 的姿态控制也很清晰：角度环输出目标角速度，角速度环输出三轴控制量，见 `drone/project/code/fly_ctrl.c:182` 到 `drone/project/code/fly_ctrl.c:210`；之后在 `Flight_Motor_Mix()` 中做 X 型四旋翼混控和限幅，见 `drone/project/code/fly_ctrl.c:218` 到 `drone/project/code/fly_ctrl.c:243`。

无名飞控可补充的是 `yaw_fault_check()`：当 yaw 控制输出很大但陀螺响应不足持续一段时间时，重置 yaw 积分和目标，见 `../无名飞控/fc_driver/control/attitude_ctrl.c:266` 到 `../无名飞控/fc_driver/control/attitude_ctrl.c:325`。这类检测对当前 drone 很有价值，因为现场问题里出现过电机突然加速、机体倾倒；如果某轴目标很大但姿态/角速度响应异常，继续积分只会扩大风险。

建议先做轻量版：

- 对 roll/pitch/yaw 分别记录“控制输出大 + 实测角速度小/方向反”的持续时间。
- 超过阈值时清对应角速度环积分，目标姿态回到当前姿态或回平，并触发无线/有线诊断标记。
- 对 yaw 目标加入 wrap/限幅工具，避免跨越边界时误差过大。

## 7. 位置控制、视觉控制和目标重捕获

无名飞控的 `position_ctrl.c` 对当前视觉跟踪最有参考价值：

- `flight_speed_remap()` 把摇杆输入平方化，低速更细腻，见 `../无名飞控/fc_driver/control/position_ctrl.c:61`。
- `get_stopping_point_xy()` 等水平速度低于阈值并持续约 200ms 后才认定停稳点，见 `../无名飞控/fc_driver/control/position_ctrl.c:73`。
- `from_enu_to_body_frame()` / `from_body_to_enu_frame()` 集中管理地球系和机体系转换，见 `../无名飞控/fc_driver/control/position_ctrl.c:105` 到 `../无名飞控/fc_driver/control/position_ctrl.c:111`。
- 光流位置控制会在固定条件不满足、模式切换或强制刹车时重置 hold 目标，见 `../无名飞控/fc_driver/control/position_ctrl.c:702` 到 `../无名飞控/fc_driver/control/position_ctrl.c:865`。

当前 `drone` 的视觉位置控制直接使用下视图像得到的小车坐标，按 yaw 快照转为地球系误差，再输出目标 roll/pitch，见 `drone/project/code/image_ctrl.c:87` 到 `drone/project/code/image_ctrl.c:117`；丢目标后连续若干帧才 reset 视觉 PID，见 `drone/project/code/image_ctrl.c:253` 到 `drone/project/code/image_ctrl.c:271`。可以借鉴的增强点：

- 视觉目标丢失/跳变时，不只是 reset PID，而是区分“短丢保持”“速度衰减后重捕获”“长丢回平”。
- 将当前 `image_ctrl.c` 内散落的坐标系转换封成小函数，减少 yaw 符号和快照/实时 yaw 混用风险。
- 对 `locked_lights == 4` 这种事件态，不应和普通可跟踪目标完全等价；需要单独的进入、保持和退出语义。
- 小车位置前馈 `Car_Position_Predict_Feedforward()` 已经有平滑变化限制，见 `drone/project/code/image_ctrl.c:27` 到 `drone/project/code/image_ctrl.c:67`，后续可以接入小车端实际速度或视觉历史拟合，而不是固定速度假设。

## 8. 光流、SLAM 和观测延迟补偿

当前 drone 主要依赖下视视觉识别小车/信标，没有独立光流。无名飞控的光流链路仍值得借鉴其观测处理方式：

- 光流解析会记录 `valid` 到 `ctrl_valid` 的延迟失效，短时无效不立刻让控制彻底失效，见 `../无名飞控/fc_driver/driver/drv_opticalflow.c:84` 到 `../无名飞控/fc_driver/driver/drv_opticalflow.c:106`。
- `opticalflow_pretreat()` 把像素流转成角速度，按质量标志判定有效，再做低通和陀螺旋转补偿，见 `../无名飞控/fc_driver/driver/drv_opticalflow.c:159` 到 `../无名飞控/fc_driver/driver/drv_opticalflow.c:176`。
- `third_order_complementarity()` 用地面高度把光流角速度变成速度观测，并用位置/速度/加速度修正项融合，见 `../无名飞控/fc_driver/algorithm/sins.c:440` 到 `../无名飞控/fc_driver/algorithm/sins.c:490`。

对 `drone` 的启发：

- 下视视觉也可以借鉴 `valid -> ctrl_valid` 两级有效性：检测有效和控制可用不是同一个概念。
- 对视觉坐标引入短历史备份，用“观测发生时刻”的姿态/高度来解算，而不是后续实时值。
- 对短时丢帧允许控制保持，但保持时必须限制时间、速度和积分。

## 9. 电机混控与抗饱和

当前 `drone` 在 `Flight_Motor_Mix()` 中手写 X4 混控并逐电机限幅，见 `drone/project/code/fly_ctrl.c:218` 到 `drone/project/code/fly_ctrl.c:243`。无名飞控的 `control_output_mix.c` 更通用：根据机架类型建立分配矩阵和伪逆矩阵 `INV_X_SCALE`，见 `../无名飞控/fc_driver/control/control_output_mix.c:78` 到 `../无名飞控/fc_driver/control/control_output_mix.c:86`；`Motor_Control_Rate_Pure()` 将 throttle/roll/pitch/yaw 映射到每个电机，见 `../无名飞控/fc_driver/control/control_output_mix.c:177` 到 `../无名飞控/fc_driver/control/control_output_mix.c:191`。

短期不建议为了 X4 直接重构矩阵混控；但可以先借鉴两个小点：

- 做统一抗饱和：当一个电机饱和时，按比例压缩三轴控制量，尽量保留控制方向。
- 记录每个轴的饱和次数，作为调参和事故分析依据。

## 10. 数学工具与安全函数

无名飞控 `wp_math.c` 中有一批飞控常见的安全数学函数：

- `safe_asin()` 对 NaN 和越界输入返回安全角度，见 `../无名飞控/fc_driver/algorithm/wp_math.c:69`。
- `safe_sqrt()` 对负数和 NaN 做保护，见 `../无名飞控/fc_driver/algorithm/wp_math.c:89`。
- `constrain_float()` 明确处理 NaN，见 `../无名飞控/fc_driver/algorithm/wp_math.c:143` 到 `../无名飞控/fc_driver/algorithm/wp_math.c:147`。
- 标准差、均值、GPS 距离、磁偏角查询等工具见 `../无名飞控/fc_driver/algorithm/wp_math.c:984`、`../无名飞控/fc_driver/algorithm/wp_math.c:1039`、`../无名飞控/fc_driver/algorithm/wp_math.c:1221`。

当前 `drone` 在很多地方直接使用 `sqrtf/atan2f/asinf`。建议至少把 `safe_sqrt/safe_asin/constrain_float` 这类工具集中到一个公共数学模块，供 IMU、TOF、视觉投影和控制限幅共用。

## 11. GPS EKF 和室外导航

当前任务主要是室内下视视觉跟车，GPS 不是近期核心。但无名飞控 GPS 部分可作为长期参考：

- `gps_homepoint_init()` 要求 3D fix、卫星数和水平精度持续满足条件后才初始化 home 点，见 `../无名飞控/fc_driver/algorithm/sins.c:676` 到 `../无名飞控/fc_driver/algorithm/sins.c:708`。
- `gps_fix_health()` 和 `gps_fusion_break()` 用不同阈值区分“定位健康”和“是否必须退出融合”，见 `../无名飞控/fc_driver/algorithm/sins.c:767` 到 `../无名飞控/fc_driver/algorithm/sins.c:785`。
- `gps_ekf_update()` 根据 GPS 位置/速度精度动态缩放 R 矩阵，并用历史状态补偿 GPS 延迟，见 `../无名飞控/fc_driver/algorithm/gps_ekf.c:172` 到 `../无名飞控/fc_driver/algorithm/gps_ekf.c:273`。

如果未来 `drone` 扩展到室外或大场地，可借鉴其“home 点稳定计数 + GPS 延迟补偿 + R 动态缩放”的结构。

## 12. 参数服务器、校准和调试记录

无名飞控把很多滤波截止频率和 PID 参数从 Flash 读取，读取失败时回退默认值，见 `../无名飞控/fc_driver/algorithm/sensor.c:67` 到 `../无名飞控/fc_driver/algorithm/sensor.c:133`。控制器初始化也统一设置各类滤波参数，见 `../无名飞控/fc_driver/algorithm/pid.c:130` 到 `../无名飞控/fc_driver/algorithm/pid.c:151`。

当前 `drone` 已有调试参数和无线调参入口，但可以继续加强：

- 把“现场经常调的参数”集中列出：TOF 有效性阈值、视觉 lost 容忍帧数、state4 冷却/保持、姿态角上限、高度 PID、视觉 PID。
- 给每次关键状态上升沿记录一次简短事件：锁定/解锁、TOF 失效、视觉全丢、state4、姿态急停、主循环超时。
- 不建议把大量无线打印放进 ISR；当前把 `merge` 打印放主循环用临时标志位的方向是正确的，见 `drone/project/user/main_cm7_0.c:135` 到 `drone/project/user/main_cm7_0.c:179`。

## 近期可执行迁移清单

以下是不需要大重构、适合分批做的小改动：

1. 给 `PID_Calculate()` 和 `Nonline_PID_Calculate()` 增加 `dt` 下限/上限和 NaN 防护。
2. 给 `Flight_Control_Loop()`、`IMU_Update_Loop()`、`Flight_Hover_Control_Task()` 记录真实周期和最大耗时。
3. 给 TOF 增加最近 N 帧标准差、突变检测、连续无效计数；短时无效保持预测，长时无效触发降级。
4. 给 roll/pitch/yaw 增加故障检测：控制输出大但角速度响应异常时清积分、回平或锁定。
5. 把 `safe_asin/safe_sqrt/constrain_float` 这类安全数学函数集中化。
6. 在视觉控制中区分“检测有效”和“控制可用”，并为 `locked_state == 4` 建立独立事件态语义。
7. 增加状态上升沿事件日志：TOF invalid、state4、lost target、emergency stop、yaw fault。

## 暂不建议直接迁移的部分

- 整套 `gps_ekf.c`：当前任务不依赖 GPS，直接移植会增加调试面。
- 完整矩阵混控：当前只有 X4，短期可先补抗饱和和饱和计数。
- 完整光流驱动：没有对应硬件时不应移植驱动，但可借鉴有效性和短时保持逻辑。
- IMU 加热控制：没有加热硬件时不适用，但温度/零偏记录值得做。
- 无名飞控的所有参数默认值：机架、电机、传感器和重量不同，不能直接套用。

## 建议的后续验证方式

- 先只引入监控和日志，不改控制输出，观察一次完整追点流程中的 `dt`、TOF 标准差、姿态输出饱和次数、视觉丢帧次数。
- 对每个新保护逻辑设置“只报警不干预”的阶段，确认触发条件和现场现象一致后再接入控制。
- 所有会改变电机输出的迁移项都应能单独开关，并保留当前稳定版本参数。
