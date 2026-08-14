#ifndef _IMAGE_CTRL_H
#define _IMAGE_CTRL_H
#include "zf_common_headfile.h"

// =================== 视觉前馈宏定义 ===================
#define CAR_FF_ENABLE           1       // 开启/关闭小车方向前馈 (1:开启, 0:关闭)
#define FF_THROW_DIST_CM        130.0f  // 收到前馈角时向该方向抛出的偏移距离峰值 (cm)
#define FF_CONVERGE_MS          2500U   // 前馈偏移线性收敛到真实小车位置的时间 (ms, 交棒位置环KI)
#define FF_THROW_RAMP_MS        300U    // 抛出量斜坡上升时间 (ms): 事件后线性升至峰值,
                                        // 避免 50cm 阶跃对位置环/姿态链的冲击 (原地下坠源)
#define FF_FLOW_CORRECTION_S    0.9f    // [新增] 光流速度修正时间常数 (s): kick 事件瞬间将无人机实际
                                        // 速度(光流, cm/s)折算为位移(cm)并从抛向量中扣除, 修正既有动量
                                        // 对 kick 方向的干扰。取值≈无人机速度响应时间量级, 可调;
                                        // 光流失效(valid=0 或高度<80cm)时 filt_vel 已归零, 本项自动降级。

#define ROTATE_RECOVER_TIME 1000

// 从无人机上电yaw零点换算到小车固定地面系时减去的初始夹角。
#define VISION_INITIAL_YAW_OFFSET_DEG 180.0f  //无人机相对小车向逆时针角度时为正
#define VISION_EARTH_YAW_DEG(yaw_deg) ((yaw_deg) - VISION_INITIAL_YAW_OFFSET_DEG)

// =================== 目标丢失容忍宏定义 ===================
#define LOST_TOLERANCE_FRAMES   6       // 连续丢失多少帧后才触发 PID 复位+回平 (防单帧噪点误触发)
                                        // 100Hz 下 6 帧 ≈ 60ms, 与 50Hz 时 3 帧 ≈ 60ms 语义一致

// =================== 偏航搜索与对准宏定义 ===================
#define TARGET_ALIGN_ENABLE      0       // 开启/关闭信标对准机制 (1:开启, 0:关闭)
#define TARGET_ALIGN_DISABLE_YAW 0.0f    // 对准关闭时的固定偏航角 (保留的特殊值)

#define SEARCH_YAW_ENABLE        1       // 开启/关闭无信标时旋转搜索机制 (1:开启, 0:关闭)
#define SEARCH_START_DELAY       100     // 信标丢失后启动搜索的延时 (ms)
#define SEARCH_WAIT_TIME         1000    // 搜索角度到位后的停留时间 (ms)
#define SEARCH_TIMEOUT           10000    // 信标丢失后的搜索总超时 (ms)
#define SEARCH_YAW_SEQ_NUM       2       // 搜索序列长度
#define SEARCH_YAW_SEQ_ARRAY     {0.0f, 10.0f} // 搜索序列 (绝对偏航角)

/**
 * @brief 自动悬停控制逻辑封装
 * @note 内部处理视觉补偿、姿态设定及 PID 计算
 */
void Flight_Hover_Control_Task(void);

// =================== 前馈偏移对外接口 ===================
// 显示方向: 机体系 (0°=当前机头, 顺时针正, 已按快照偏航旋入, 与飞行实际施加一致)
void Car_Feedforward_Reset(void);         // 复位前馈偏移状态 (丢失回平/视觉失联时调用)
extern float ff_disp_dir_deg;             // 当前前馈方向角 (deg, 机体系 0°=机头, 0=无前馈) → M7_0_data_send 下传 CM7_1 屏幕绘制
extern float ff_disp_remain_cm;           // 当前前馈剩余偏移量 (cm, 0=无前馈) → 同上
extern float ff_off_x;                    // [新增] 事件时刻修正后前馈偏移向量X (cm, 地面系) → 打印函数换算角度
extern float ff_off_y;                    // [新增] 事件时刻修正后前馈偏移向量Y (cm, 地面系) → 打印函数换算角度
extern float ff_event_deg;                // [新增] 已应用前馈事件的角度 (deg, 地面系, -1=无事件) → 打印函数触发源
#endif
