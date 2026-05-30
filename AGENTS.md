# Drone Flight Control — AI Navigation

**Branch:** `sort_test` | **Commit:** `3dcd171` | **Generated:** 2026-05-19

## OVERVIEW
Dual-core Cortex-M7 drone flight control on CYT4BB platform. CM7_0 runs 1ms flight loop; CM7_1 runs 20ms image processing. Built with IAR EW 9.40.1 + SeekFree library v3.9.1. See CLAUDE.md for detailed architecture.

## STRUCTURE
```
./
├── project/code/     # All flight control logic (14 modules, 29 files) — SEE AGENTS.md there
├── project/user/     # Entry points + ISRs (4 files): main_cm7_{0,1}.c, cm7_{0,1}_isr.c
├── project/iar/      # IAR workspace, project files, linker script, debug cfg
├── libraries/        # SeekFree v3.9.1 SDK (zf_common/driver/device) + Infineon SDK
├── doc/              # Optical flow sensor reference PDF
├── CLAUDE.md         # Full architecture docs, Karpathy guidelines, parameter table
└── README.md         # Project conventions, coordinate systems, full changelog
```

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| **Flight instability** | `project/code/fly_ctrl.c` + `imu.c` | PID cascade; check IMU axis mapping in `imu.h` |
| **Visual tracking broken** | `project/code/image_process.c` + `image_ctrl.c` | Binarize→closing→DFS→sort→ground project |
| **Inter-core data wrong** | `project/code/data_complex.h` | Check `S0_*`/`S1_*` index macros, DCache ops |
| **Build fails** | `project/iar/` | Run cleanup .bat; check both cores build |
| **Motor PWM wrong** | `project/code/fly_ctrl.c` (motor_pwm_set) | Check motor mapping after frame change (5.12a) |
| **Parameter tuning** | `project/code/app.c` + `pid.c` | Keys 1/2/3; debug UART; see CLAUDE.md param table |
| **Camera/TOF init** | `project/code/imu.c` + `image.c` | while(1) loops on init fail — check hardware |
| **New module to add** | Add .c/.h to `project/code/`, include in `libraries/zf_common/zf_common_headfile.h` | Auto-picked by build |

## KEY ARCHITECTURE

### Entry + ISR Flow (project/user/)
- `main_cm7_0.c` — CM7_0 entry: init sensors → PIT_CH0@1ms → flight loop
- `cm7_0_isr.c` — `pit0_ch0_isr()`: IMU→PID→motor; uart1/3/4 ISRs
- `main_cm7_1.c` — CM7_1 entry: init camera → wait frame → image processing
- `cm7_1_isr.c` — `pit0_ch10_isr()`: keys; UART ISRs

### The One Include Rule
**Every .c file includes `"zf_common_headfile.h"`** (at `libraries/zf_common/`). This aggregates: C stdlib → Infineon SDK → SeekFree drivers/devices → ALL project headers. Do NOT add per-file includes — add to `zf_common_headfile.h` instead.

### Shared Memory (dual-core)
Two `volatile float` arrays at fixed addresses:
- `share_data_from_0[16]` @ 0x28001040 — CM7_0 writes, CM7_1 reads (IMU, state, motors)
- `share_data_from_1[16]` @ 0x28001000 — CM7_1 writes, CM7_0 reads (vision results)
- Index macros: `S0_*` / `S1_*` defined in `data_complex.h`
- **CRITICAL**: `SCB_CleanDCache_by_Addr()` after write, `SCB_InvalidateDCache_by_Addr()` before read

## CONVENTIONS
- **Naming**: PascalCase for high-level funcs (`Flight_Control_Init`), snake_case for init/low-level (`imu_init`, `motor_pwm_set`). Structs: `_t` suffix. Enums: `_e` suffix. Macros: `ALL_CAPS`.
- **Comments**: Chinese `//` style. `/**` doxygen on minority of functions. Tagged with `[新增]`/`[修改]`/`注意：`.
- **Float everywhere**: All flight control uses `float` (Cortex-M7 FPU). `double` only in `image_process.c` for ray-cast geometry and `imu.c` calibration accumulators.
- **Constants**: Always `f` suffix: `120.0f`, `0.001f`.
- **Brace style**: K&R dominant, some Allman mixed. Don't mix in same file.
- **File encoding**: UTF-8 (changed at v1.28d).

## ANTI-PATTERNS
- **`while(1)` on init fail** — `imu.c`, `image.c`, `wireless_uart.c` hang forever instead of reporting error. Don't add more of these.
- **`void` functions everywhere** — no error propagation. Only PID/Kalman return values.
- **Commented-out code** — abundant dead code left in place. Don't add more; clean up when touching nearby code.
- **Mixed naming in same module** — e.g. `M7_0_data_send()` mixes PascalCase + snake_case. Follow existing style per-file.
- **Missing `const`** — no const-correctness on pointer params. Not required to fix, but don't make it worse.
- **Typo**: `Motor_Offsset_t` (double 's') — don't propagate.
- **Unused declarations** — 声明但未使用过的变量/结构体/函数不要删除，后续可能会使用。

## COMMANDS

**Build** (Windows + IAR only):
```cmd
:: Open IDE
start project/iar/cyt4bb7.eww              :: Master workspace (both cores)

:: CLI build (IAR installed)
iarbuild.exe "project/iar/project_config/cyt4bb7_cm_7_0.ewp" -build Debug
iarbuild.exe "project/iar/project_config/cyt4bb7_cm_7_1.ewp" -build Debug
```

**Clean**: Run `project/iar/删除临时文件IAR.bat` (Windows only)

**Flash/Debug (CLI)**: `project/iar/project_config/settings/cyt4bb7_cm_7_0.Debug.cspy.bat`

## NOTES
- **No test infrastructure.** No CI/CD. Debug via UART prints + IPS200 screen.
- **`project/iar/` is gitignored** — config files (`.ewp`, `.ewd`, `.icf`) are gitignored but critical for builds. Don't lose them.
- **Two PIT timers can't be shared** between cores (v3.9a changelog).
- **TOF over-range** returns 8192 — clamp to 1400.
- **Motor mapping changed at 5.12a**: motor1↔LF, motor2↔RF, motor3↔RB, motor4↔LB. LF/RB CW, RF/LB CCW.
- **Debug mode**: Switch 1 OFF → motors disabled but all logic runs (debug output visible on screen).
- **Flight mode**: Switch 1 ON → full control with motors active.
