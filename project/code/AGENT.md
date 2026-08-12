# project/code/ — Flight Control Modules

All drone flight control, image processing, PID, and communication logic. 14 modules, 29 files. Every `.c` includes `zf_common_headfile.h` as its sole include.

## MODULE MAP

| Module (.c/.h) | Lines | Core Role |
|---|---|---|
| **fly_ctrl** | 277+.h/.c | PID cascade: position→angle→rate. Motor mixing + PWM output. State machine (normal/pre_landing/landing). |
| **imu** | 366+.h/.c | Mahony sensor fusion. TOF height. Complementary filtering. Axis mapping macros. Gyro calibration. |
| **pid** | 138+.h/.c | Nonlinear PID (kp2 squared term). Linear PID. Anti-windup. D-term LPF. 9 global instances. |
| **image** | 528+.h/.c | Camera init (MT9V03X). Image processing orchestration. Distance estimation. Light detection loop. |
| **image_process** | 186+.h/.c | Binarize (per-pixel dynamic threshold). Double closing (dilate→erode→dilate→erode). DFS connected components. Light sorting. Ground projection (ray-cast, double precision). |
| **image_ctrl** | 198+.h/.c | Visual hover control. Target tracking. Search pattern (yaw rotation). Memory on target loss. Vision PID loop. |
| **data_complex** | 133+.h/.c | Inter-core shared arrays (`S0_*`/`S1_*`). Board UART comms (0xAA/0x55/0x7F framing). `Data_Complex_t` struct. |
| **app** | 232+.h/.c | State machine (DEBUG/FLIGHT). Parameter tuning UI (keys 1/2/3). Watchdog reset. |
| **kalman_filter** | 25+.h/.c | 1D Kalman for car position (X/Y independent). Tiny module — ~50 lines total. |
| **key_switch** | 183+.h/.c | Button debounce + event extraction. Long/short press detection. Switch state. |
| **display** | 184+.h/.c | IPS200 screen debug output. Motor, IMU, PID, position displays. |
| **upixel** | 118+.h/.c | Optical flow sensor (LC-302). Byte-level parser. Velocity calculation. |
| **wireless_uart** | 146+.h/.c | Wireless UART debug output. Motor/PID data telemetry. |
| **small_driver_uart_control** | 219+.h/.c | Motor ESC UART protocol. 4-motor PWM-to-serial conversion. Motor arm/disarm. |

## WHERE TO LOOK

| Problem | Start Here |
|---|---|
| Drone unstable/oscillating | `fly_ctrl.c` — PID cascade gains. `pid.c` — kp/ki/kd values. `app.c` — parameter tuning. |
| IMU data wrong / orientation off | `imu.h` — axis mapping macros (`IMU_MAP_GX`, `IMU_MAP_AX`). `imu.c` — Mahony filter. |
| Camera not detecting lights | `image.h` — threshold constants. `image_process.c` — binarization, DFS. |
| Visual tracking drifts | `image_ctrl.c` — hover control. `image_process.c` — ground projection (calibration). |
| Motor wrong direction / mapping | `fly_ctrl.c` — `motor_pwm_set()` mapping after frame change (5.12a). |
| Inter-core data stale/corrupt | `data_complex.h` — S0/S1 indices. DCache sync in `main_cm7_{0,1}.c`. |
| Parameter tuning not working | `app.c` — `Fly_Param_Update()`. `key_switch.c` — event parsing. |
| Build fails (new .c file) | Ensure added to `zf_common_headfile.h` — auto-compiled by IAR. |

## DATA FLOW (simplified)
```
CM7_0:
  PIT_CH0 1ms: tof_update + pit0_cnt
  PIT_CH1 1.25ms: imu.c:Mahony_Update → fly_ctrl.c:Flight_Control_Loop → motor_pwm_set
  main_cm7_0 (main loop): reads share_data_from_1 → image_ctrl.c:Flight_Hover_Control_Task → writes share_data_from_0

CM7_1 (100Hz camera):
  image.c:main → image_process.c:pipeline → image_ctrl.c → data_complex.c:M7_1_data_send → share_data_from_1
```

## MODULE CONVENTIONS

- **`static` for internal functions**: `fly_ctrl.c` uses `static` for all PID helpers (`Flight_State_Update`, `Flight_Control_Height`, `Flight_Control_Rate`, `Flight_Motor_Mix`). Follow this pattern.
- **Doxygen `/**` on public API**: Used in `fly_ctrl.c`, `pid.h`, `upixel.c`, `image.c`. Add for new public functions.
- **`[新增]`/`[修改]` tags**: Chinese change annotations on new/modified lines. Use when touching code.
- **Section dividers**: `// ======` for config blocks, `// ----` for logic sections. Follow existing file style.
- **File-scope globals**: Declare in `.h` with `extern`, define in `.c`. Shared arrays in `data_complex.c` use `#pragma location`.
- **New module**: Add `#include "module.h"` to `zf_common_headfile.h`. IAR auto-compiles everything in this directory.
