#ifndef _CONFIG_H
#define _CONFIG_H

// ================== 起飞模式配置 ==================
#define AUTO_TAKEOFF_ENABLE     1       // 自动起飞使能开关 (1: 现有自动起飞高度缓升方式, 0: 关闭自动起飞, start_up_scale 直接缩放最终 PWM 输出)

#define STRATEGY (2)

#if STRATEGY == 1
// ================== 视觉前馈参数 ==================
// FF_THROW_RAMP_MS: 抛出量斜坡上升时间, 避免阶跃对位置环/姿态链的冲击 (原地下坠源)
// FF_FLOW_CORRECTION_S: 光流速度修正时间常数, kick 瞬间将光流速度折算为位移并从抛向量中扣除; 光流失效时自动降级
#define CAR_FF_ENABLE           1       // 开启/关闭小车方向前馈 (1:开启, 0:关闭)
#define FF_THROW_DIST_CM        150.0f  // 收到前馈角时向该方向抛出的偏移距离峰值 (cm)
#define FF_CONVERGE_MS          2500U   // 前馈偏移线性收敛到真实小车位置的时间 (ms, 交棒位置环KI)
#define FF_THROW_RAMP_MS        300U    // 抛出量斜坡上升时间 (ms): 事件后线性升至峰值
#define FF_FLOW_CORRECTION_S    1.1f    // 光流速度修正时间常数 (s)
#endif

#if STRATEGY == 2
// ================== 视觉前馈参数 ==================
// FF_THROW_RAMP_MS: 抛出量斜坡上升时间, 避免阶跃对位置环/姿态链的冲击 (原地下坠源)
// FF_FLOW_CORRECTION_S: 光流速度修正时间常数, kick 瞬间将光流速度折算为位移并从抛向量中扣除; 光流失效时自动降级
#define CAR_FF_ENABLE           1       // 开启/关闭小车方向前馈 (1:开启, 0:关闭)
#define FF_THROW_DIST_CM        230.0f  // 收到前馈角时向该方向抛出的偏移距离峰值 (cm)
#define FF_CONVERGE_MS          2000U   // 前馈偏移线性收敛到真实小车位置的时间 (ms, 交棒位置环KI)
#define FF_THROW_RAMP_MS        300U    // 抛出量斜坡上升时间 (ms): 事件后线性升至峰值
#define FF_FLOW_CORRECTION_S    2.1f    // 光流速度修正时间常数 (s)
#endif

#if STRATEGY == 3
// ================== 视觉前馈参数 ==================
// FF_THROW_RAMP_MS: 抛出量斜坡上升时间, 避免阶跃对位置环/姿态链的冲击 (原地下坠源)
// FF_FLOW_CORRECTION_S: 光流速度修正时间常数, kick 瞬间将光流速度折算为位移并从抛向量中扣除; 光流失效时自动降级
#define CAR_FF_ENABLE           1       // 开启/关闭小车方向前馈 (1:开启, 0:关闭)
#define FF_THROW_DIST_CM        140.0f  // 收到前馈角时向该方向抛出的偏移距离峰值 (cm)
#define FF_CONVERGE_MS          2200U   // 前馈偏移线性收敛到真实小车位置的时间 (ms, 交棒位置环KI)
#define FF_THROW_RAMP_MS        300U    // 抛出量斜坡上升时间 (ms): 事件后线性升至峰值
#define FF_FLOW_CORRECTION_S    1.1f    // 光流速度修正时间常数 (s)
#endif

#endif // _CONFIG_H
