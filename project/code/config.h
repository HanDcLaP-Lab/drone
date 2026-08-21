#ifndef _CONFIG_H
#define _CONFIG_H

// ================== 初始姿态零偏与电机基础悬停油门 ==================
// 可将空中校准完成后无线串口打印的数值填入此处，后续启动即可开机自带精准物理零点
#define INIT_ROLL_OFFSET_DEG        -0.28f    // 初始横滚零点偏移 (度, 填入空中校准打印的 roll_offset)
#define INIT_PITCH_OFFSET_DEG       1.77f    // 初始俯仰零点偏移 (度, 填入空中校准打印的 pitch_offset)

#define INIT_HOVER_PWM_LF           5369    // 左前电机 (LF) 初始悬停油门
#define INIT_HOVER_PWM_RF           4996    // 右前电机 (RF) 初始悬停油门
#define INIT_HOVER_PWM_LB           5048    // 左后电机 (LB) 初始悬停油门
#define INIT_HOVER_PWM_RB           5369    // 右后电机 (RB) 初始悬停油门

#define STRATEGY (1)

#if STRATEGY == 1
// ================== 视觉前馈参数 ==================
// FF_THROW_RAMP_MS: 抛出量斜坡上升时间, 避免阶跃对位置环/姿态链的冲击 (原地下坠源)
// FF_FLOW_CORRECTION_S: 光流速度修正时间常数, kick 瞬间将光流速度折算为位移并从抛向量中扣除; 光流失效时自动降级
// FF_REVERSE_GAIN: 掉头/大角度反向增益系数 (90~180度夹角线性放大响应幅值, 1.0=保持当前策略)
#define CAR_FF_ENABLE           1       // 开启/关闭小车方向前馈 (1:开启, 0:关闭)
#define FF_THROW_DIST_CM        170.0f  // 收到前馈角时向该方向抛出的偏移距离峰值 (cm)
#define FF_CONVERGE_MS          2500U   // 前馈偏移线性收敛到真实小车位置的时间 (ms, 交棒位置环KI)
#define FF_THROW_RAMP_MS        300U    // 抛出量斜坡上升时间 (ms): 事件后线性升至峰值
#define FF_FLOW_CORRECTION_S    0.0f    // 光流速度修正时间常数 (s)
#define FF_REVERSE_GAIN         1.0f    // 掉头/大角度反向增益系数 (90~180度线性放大, 1.0=保持当前策略)
#endif

#if STRATEGY == 2
// ================== 视觉前馈参数 ==================
// FF_THROW_RAMP_MS: 抛出量斜坡上升时间, 避免阶跃对位置环/姿态链的冲击 (原地下坠源)
// FF_FLOW_CORRECTION_S: 光流速度修正时间常数, kick 瞬间将光流速度折算为位移并从抛向量中扣除; 光流失效时自动降级
// FF_REVERSE_GAIN: 掉头/大角度反向增益系数 (90~180度夹角线性放大响应幅值, 1.0=保持当前策略)
#define CAR_FF_ENABLE           1       // 开启/关闭小车方向前馈 (1:开启, 0:关闭)
#define FF_THROW_DIST_CM        250.0f  // 收到前馈角时向该方向抛出的偏移距离峰值 (cm)
#define FF_CONVERGE_MS          2400U   // 前馈偏移线性收敛到真实小车位置的时间 (ms, 交棒位置环KI)
#define FF_THROW_RAMP_MS        300U    // 抛出量斜坡上升时间 (ms): 事件后线性升至峰值
#define FF_FLOW_CORRECTION_S    1.5f    // 光流速度修正时间常数 (s)
#define FF_REVERSE_GAIN         1.0f    // 掉头/大角度反向增益系数 (90~180度线性放大, 1.0=保持当前策略)
#endif

#if STRATEGY == 3
// ================== 视觉前馈参数 ==================
// FF_THROW_RAMP_MS: 抛出量斜坡上升时间, 避免阶跃对位置环/姿态链的冲击 (原地下坠源)
// FF_FLOW_CORRECTION_S: 光流速度修正时间常数, kick 瞬间将光流速度折算为位移并从抛向量中扣除; 光流失效时自动降级
// FF_REVERSE_GAIN: 掉头/大角度反向增益系数 (90~180度夹角线性放大响应幅值, 1.0=保持当前策略)
#define CAR_FF_ENABLE           1       // 开启/关闭小车方向前馈 (1:开启, 0:关闭)
#define FF_THROW_DIST_CM        100.0f  // 收到前馈角时向该方向抛出的偏移距离峰值 (cm)
#define FF_CONVERGE_MS          2400U   // 前馈偏移线性收敛到真实小车位置的时间 (ms, 交棒位置环KI)
#define FF_THROW_RAMP_MS        300U    // 抛出量斜坡上升时间 (ms): 事件后线性升至峰值
#define FF_FLOW_CORRECTION_S    1.0f    // 光流速度修正时间常数 (s)
#define FF_REVERSE_GAIN         1.5f    // 掉头/大角度反向增益系数 (90~180度线性放大, 1.0=保持当前策略)
#endif

#endif // _CONFIG_H
