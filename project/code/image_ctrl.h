#ifndef _IMAGE_CTRL_H
#define _IMAGE_CTRL_H
#include "zf_common_headfile.h"

// =================== 视觉前馈宏定义 ===================
#define CAR_FF_ENABLE           0       // 开启/关闭视觉前馈 (1:开启, 0:关闭)
#define CAR_FF_DIST_THRESHOLD   50.0f    // 预测触发距离阈值 (cm)
#define CAR_FF_SPEED            40.0f    // 小车运动速度 (cm/s, 即0.4m/s)
#define CAR_FF_PREDICT_TIME     0.25f    // 预测前瞻时间 (s) 结合机械延迟建议0.25s起步
#define CAR_FF_MAX_CHANGE       2.0f     // 每次循环前馈增量的最大变化限制 (cm)

#define ROTATE_RECOVER_TIME 1000

// 从无人机上电yaw零点换算到小车固定地面系时减去的初始夹角。
#define VISION_INITIAL_YAW_OFFSET_DEG 0.0f  //无人机相对小车向逆时针角度时为正
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
#endif
