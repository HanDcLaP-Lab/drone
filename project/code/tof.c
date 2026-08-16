#include "tof.h"
#include "imu.h"                      // imu_data (z, vz, roll, pitch)
#include <math.h>
#include "fly_ctrl.h"                 // flight_target

#if TOF_SENSOR_VL53L8CX
#include "vl53l8cx/platform.h"        // SPI 引脚宏 / platform 函数
#if VL53L8CX_USE_XTALK_CALIB
#include "vl53l8cx_plugin_xtalk.h"
#include "vl53l8cx_xtalk_calib_data.h"
#endif
#endif

// ================== 全局变量定义 ==================
float    tof_actual_dt    = 0.02f;          // 实测 TOF 帧间隔 (秒), 调试用
int16_t  tof_base_throttle = 0; // 高度 PID 修正量 (不含悬停基准)
float    z_rate           = 0;               // 调试: 高度位置环输出 (目标爬升率 cm/s)
float    z_acc            = 0;               // 调试: 高度速度环输出 (油门增量)
volatile uint32_t tof_update_seq = 0;

uint16_t tof_cnt = 0;
// ================== TOF 帧间差分状态 (双传感器共用) ==================
static float    tof_z_prev    = 0.0f;
static float    tof_vz_filt   = 0.0f;
static float    tof_z_filt    = 0.0f;       // Z位置低通滤波状态
static uint8_t  tof_has_prev  = 0;
static uint32_t tof_last_ready_tick = 0;  // 复用 dataC.pit0_cnt 做 dt 测量

#if TOF_SENSOR_VL53L8CX
VL53L8CX_Configuration  vl53l8cx_dev;
VL53L8CX_ResultsData    vl53l8cx_results;
volatile uint8_t        vl53l8cx_data_ready = 0;

// ================== VL53L8CX 16 区修剪均值 ==================
static uint16_t VL53L8CX_Trimmed_Mean_MM(void) {
    int16_t valid_dist[16];
    int     valid_cnt = 0;

    for (int i = 0; i < 16; i++) {
        uint8_t st = vl53l8cx_results.target_status[i];
        int16_t d  = vl53l8cx_results.distance_mm[i];
        if ((st == 5 || st == 9) && d >= 30 && d <= 4000)
            valid_dist[valid_cnt++] = d;
    }

    if (valid_cnt == 0) return 0;

    // 冒泡排序
    for (int i = 0; i < valid_cnt - 1; i++)
        for (int j = 0; j < valid_cnt - 1 - i; j++)
            if (valid_dist[j] > valid_dist[j+1]) {
                int16_t tmp = valid_dist[j];
                valid_dist[j] = valid_dist[j+1]; valid_dist[j+1] = tmp;
            }

    // 计算小端/大端舍弃个数
    int trim_lo = 0, trim_hi = 0;
    if (TOF_TRIM_LO_PCT > 0) {
        trim_lo = valid_cnt * TOF_TRIM_LO_PCT / 100;
        if (trim_lo < 1) trim_lo = 1;
    }
    if (TOF_TRIM_HI_PCT > 0) {
        trim_hi = valid_cnt * TOF_TRIM_HI_PCT / 100;
        if (trim_hi < 1) trim_hi = 1;
    }

    int remaining = valid_cnt - trim_lo - trim_hi;

    if (remaining > 2) {
        // 修剪均值
        int sum = 0;
        for (int i = trim_lo; i < valid_cnt - trim_hi; i++)
            sum += valid_dist[i];
        return (uint16_t)(sum / remaining);
    } else {
        // 有效点太少，回退到中位数
        return (uint16_t)valid_dist[valid_cnt / 2];
    }
}
#endif

// ================== 内部: 倾角补偿 + VZ 计算 + 高度 PID (双传感器共用) ==================
// dt = 两次有效数据之间的实测间隔 (秒)
static void tof_process_z(float raw_mm, float dt) {
    // 1. 无效数据保护
    if (raw_mm < 10) return;

    // 2. 物理限幅
    if (raw_mm > 1500) raw_mm = 1500;

    // 3. 倾角补偿
    float rad_roll  = imu_data.roll  * (PI / 180.0f);
    float rad_pitch = imu_data.pitch * (PI / 180.0f);
    float kc = fabsf(cosf(rad_roll) * cosf(rad_pitch));
    float height_cm = (raw_mm / 10.0f) * kc;

    // 4. Z 位置低通滤波 (EMA, alpha=0.2, τ≈100ms @50Hz)
    //    滤除 TOF 帧间量化噪声，输出稳定高度供 PID 和外部模块使用
    if (tof_has_prev) {
        tof_z_filt += (height_cm - tof_z_filt) * 0.2f;
    } else {
        tof_z_filt = height_cm;  // 首帧直接初始化，不做渐变
    }
    imu_data.z = tof_z_filt;

    if (tof_has_prev && imu_data.z > 10.0f) {
        float vz_raw = (tof_z_filt - tof_z_prev) / dt;
        tof_vz_filt += (vz_raw - tof_vz_filt) * 0.2f;
        imu_data.vz = tof_vz_filt;
    }
    tof_z_prev = tof_z_filt;  // 记录本帧滤波值供下帧 VZ 差分
    tof_has_prev = 1;

    // 6. 高度 PID (位置环 → 速度环 → 基础油门, 不含倾角补偿)
    //    全部使用滤波后的数据，高度环不需要响应速度、需要稳定
    float height_error = flight_target.height - imu_data.z;
    float target_climb_rate = PID_Calculate(&pid_height_pos, height_error, dt);
    z_rate = target_climb_rate;
    float climb_rate_error = target_climb_rate - imu_data.vz;
    float throttle_adj = PID_Calculate(&pid_height_vel, climb_rate_error, dt);
    z_acc = throttle_adj;
    tof_base_throttle = (int16_t)throttle_adj;
    tof_update_seq++;
}

// ================== tof_init ==================
void tof_init(void){
#if TOF_SENSOR_VL53L8CX
    // 1. SPI 初始化 (手动 CS)
    spi_init(VL53L8CX_SPI_IDX, SPI_MODE3, VL53L8CX_SPI_BAUDRATE,
             VL53L8CX_SPI_CLK, VL53L8CX_SPI_MOSI, VL53L8CX_SPI_MISO, SPI_CS_NULL);

    vl53l8cx_dev.platform.spi_n     = VL53L8CX_SPI_IDX;
    vl53l8cx_dev.platform.cs_pin    = VL53L8CX_CS_PIN;
    // xshut_pin 不赋值 — LPn 硬件接高电平

    gpio_init(VL53L8CX_CS_PIN,  GPO, GPIO_HIGH, GPO_PUSH_PULL);
    // gpio_init(VL53L8CX_INT_PIN,   GPI, 0,         GPI_FLOATING_IN);
    // exti_init(VL53L8CX_INT_PIN, EXTI_TRIGGER_FALLING); // INT 低有效 → 下降沿
    // ↑ INT 引脚改用 SPI 轮询 (排线串扰会导致伪中断)

    // 2. 检测传感器是否存在
    while (1) {
        uint8_t is_alive = 0;
        vl53l8cx_is_alive(&vl53l8cx_dev, &is_alive);
        if (is_alive) {
            printf("VL53L8CX detected\n");
            break;
        }
        printf("VL53L8CX not found, retry...\n");
        system_delay_ms(1000);
    }

    // 3. 初始化 (下载固件 ~70ms @10MHz SPI)
    uint8_t status = vl53l8cx_init(&vl53l8cx_dev);
    if (status) printf("VL53L8CX init error: %d\n", status);
    else         printf("VL53L8CX init done\n");

    // 4. 配置 4x4 @50Hz 连续模式
    vl53l8cx_set_resolution(&vl53l8cx_dev, VL53L8CX_RESOLUTION_4X4);
    vl53l8cx_set_ranging_frequency_hz(&vl53l8cx_dev, 50);
    vl53l8cx_set_ranging_mode(&vl53l8cx_dev, VL53L8CX_RANGING_MODE_CONTINUOUS);
    vl53l8cx_set_target_order(&vl53l8cx_dev, VL53L8CX_TARGET_ORDER_CLOSEST);

    // 5. 若启用 xtalk 校准数据, 则加载后开始测距
#if VL53L8CX_USE_XTALK_CALIB
    vl53l8cx_set_caldata_xtalk(&vl53l8cx_dev, (uint8_t*)vl53l8cx_xtalk_calib_data);
    printf("VL53L8CX xtalk calib data loaded\n");
#endif
    vl53l8cx_start_ranging(&vl53l8cx_dev);
    printf("VL53L8CX ranging started\n");
#else
    // ========== DL1B 原逻辑 ==========
    while(1)
    {
        if(dl1b_init())
            printf("tof_init_error");
        else
            {
                printf("tof_init_done");
                break;

            }
        system_delay_ms(1000);
    }
#endif
}

// ================== tof_update ==================
void tof_update(void) {
#if TOF_SENSOR_VL53L8CX
    // ---- VL53L8CX: SPI 轮询 check_data_ready (排线串扰导致 INT 引脚伪中断) ----
    {
        uint8_t ready = 0;
        vl53l8cx_check_data_ready(&vl53l8cx_dev, &ready);
        if (!ready) return;
    }

    vl53l8cx_get_ranging_data(&vl53l8cx_dev, &vl53l8cx_results);

    uint16_t raw_mm = VL53L8CX_Trimmed_Mean_MM();
    if (raw_mm == 0) return;

    // 实测 dt (基于 1ms tick 计数器)
    uint16_t dt_ticks = dataC.pit0_cnt - tof_last_ready_tick;
    float dt = dt_ticks * 0.001f;
    //if (dt < 0.005f) dt = 0.02f;  // 首帧/异常: 回退到标称 20ms
    if (dt > 0.1f)   dt = 0.1f;   // 超时上限: 10Hz 等效
    tof_last_ready_tick = dataC.pit0_cnt;
    tof_actual_dt = dt;

    tof_process_z((float)raw_mm, dt);

    
    tof_cnt++;
#else
    // ---- DL1B: 数据就绪标志触发 → TOF-only 处理 (不再依赖 IMU 加速度) ----
    dl1b_get_distance();
    if (dl1b_finsh_flag == 1) {
        dl1b_finsh_flag = 0;

        // 实测 dt (基于 1ms tick 计数器)
        uint16_t dt_ticks = dataC.pit0_cnt - tof_last_ready_tick;
        float dt = dt_ticks * 0.001f;
        if (dt < 0.005f) dt = 0.02f;  // 首帧/异常: 回退到标称值
        if (dt > 0.1f)   dt = 0.1f;   // 超时上限
        tof_last_ready_tick = dataC.pit0_cnt;
        tof_actual_dt = dt;

        tof_process_z((float)dl1b_distance_mm, dt);
    }
#endif
}

