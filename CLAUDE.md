# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview
This is a multi-core Cortex-M7 drone flight control system for the CYT4BB platform. The drone uses dual CM7 cores (CM7_0 and CM7_1) for flight control and image processing respectively. The project is built with IAR Embedded Workbench and uses the "逐飞科技" (SeekFree) open-source library (version 3.9.1).

## Core Architecture

### Dual-Core Communication
- **CM7_0**: Main flight control core (1ms control loop)
  - Sensor fusion (IMU, TOF)
  - PID control loops (visual → angle → angular velocity)
  - Motor PWM output
  - Interrupt handling (PIT_CH0: 1ms)

- **CM7_1**: Image processing core (20ms processing loop)  
  - MT9V03X camera processing (120x188 resolution)
  - Per-pixel dynamic threshold: center 130 → edge 40 (3×3 block LUT)
  - Pipeline: binarize → double closing (dilate→erode→dilate→erode) → DFS components → sort → ground projection
  - Visual tracking (car and beacon recognition)
  - Interrupt handling (PIT_CH1: 20ms)

**Inter-core communication:**
- Two shared data arrays: `share_data_from_0[16]` (CM7_0 → CM7_1) and `share_data_from_1[16]` (CM7_1 → CM7_0)
- Index macros defined in `data_complex.h`: `S0_*` (e.g. `S0_IMU_ROLL=0`) and `S1_*` (e.g. `S1_K_CAR_X=9`)
- Data synchronization via `SCB_CleanDCache_by_Addr()`
- Structure: `Data_Complex_t` in `data_complex.h` for managing complex data

### Control System Architecture
The drone uses a triple-cascade PID structure:
1. **Visual Loop**: Processes camera data to generate XY position error
2. **Angle Loop**: Converts position error to target roll/pitch angles  
3. **Angular Velocity Loop**: Converts angle targets to motor outputs

**Key modules:**
- `fly_ctrl.c/h`: Main flight control with `Flight_Target_t` struct for targets
- `imu.c/h`: IMU data processing and coordinate transforms
- `image.c/h`: Image processing — binarize (per-pixel threshold), double closing, DFS components, light sorting, ground projection via `image_process.c`  
- `pid.c/h`: Nonlinear PID implementation with filtering
- `upixel.c/h`: Optical flow module (currently using screen UART)
- `app.c/h`: State machine with debug/flight mode switching

### Coordinate System Definitions
- **Ground reference**: Drone projection as origin
  - X: Forward direction projection on ground
  - Y: Right side direction projection on ground
- **Drone body**: IMU installation determines axes
  - X: Horizontal forward
  - Y: Horizontal right  
  - Z: Vertically downward
- **Camera coordinates**: Top-left (0,0), rightward Y, downward X

## Build System

### Development Environment
- **IDE**: IAR Embedded Workbench 9.40.1
- **Workspace**: `project/iar/cyt4bb7.eww`
- **Project files**: 
  - `project/iar/project_config/cyt4bb7_cm_7_0.ewp` (CM7_0)
  - `project/iar/project_config/cyt4bb7_cm_7_1.ewp` (CM7_1)

### Build Commands
- Open `project/iar/cyt4bb7.eww` in IAR IDE
- Select workspace target (CM7_0 or CM7_1)
- Use project→build or F7 key
- Clean temporary files: `project/iar/删除临时文件IAR.bat`

### Debug Configuration
- Workspace uses separate debug configurations for each core
- Debug interface via CSPY bat files in `project/iar/project_config/settings/`

## Key Configuration Constants

### Flight Parameters (fly_ctrl.h)
- `TARGET_HEIGHT_CM`: 120.0f (target height in cm)
- `HOVER_THROTTLE`: 4800 (base hover PWM value)
- `MAX_TILT_ANGLE`: 6.0f ° (maximum tilt for normal flight)
- `CTRL_DT_CTLOOP`: 0.001s (1ms control cycle)
- `SEARCH_YAW_RATE`: 15.0f °/s (search rotation speed)

### Camera Configuration  
- `MT9V03X_H`: 120, `MT9V03X_W`: 188 (resolution)
- `CAM_OFFSET_X`: -9.0f, `CAM_OFFSET_Y`: -2.0f, measured in body axes at `CAM_OFFSET_MEASURE_YAW_DEG`: 0.0f; converted once to the car-fixed ground frame
- `THRESHOLD_MAX`: 130 (image center), `THRESHOLD_MIN`: 120 (image edge) — per-pixel via rho² LUT
- `BASE_MIN_AREA`: 1.0f — minimum blob area for valid light detection (currently so low it acts as a passthrough)
- `FOV_DIAMETER`: 125.0f — circular FOV mask diameter (pixels)

### Motor Mapping
New frame (post 5.12a):
- motor1 ↔ LF (left front)
- motor2 ↔ RF (right front)  
- motor3 ↔ RB (right back)
- motor4 ↔ LB (left back)

**Rotation directions**: LF and RB motors clockwise; RF and LB motors counterclockwise

## Development Workflow

### Code Organization
- User code goes in `project/code/` (automatically picked up by build system)
- Core application files:
  - `project/user/main_cm7_0.c`: CM7_0 entry point with 1ms interrupt
  - `project/user/main_cm7_1.c`: CM7_1 entry point with 20ms interrupt

### Testing and Debugging
- **Mode selection**: Set `DRONE_MODE_SELECT` to switch-controlled, forced debug, or forced normal mode
- **Forced normal mode**: Full control is enabled after IMU calibration; forced debug keeps motors disabled
- **Parameter tuning**: Connected debug UART for parameter updates
- **Screen display**: IPS200 screen for debugging info via `ips200_show_xxx()`

### Parameter Tuning Process
1. Use `Fly_Param_Update()` functions in `app.c` via debug interface
2. Buttons Key2 (increase 5%) and Key3 (decrease 5%) for manual tuning
3. Key1 to cycle between different PID parameters
4. Visual guidance: `/find-skills` command in Claude Code (manual implementation)

## Important Notes for Code Changes

### Interrupt Safety
- CM7_0 uses PIT_CH0 (1ms) for flight control
- CM7_1 uses PIT_CH1 (20ms) for image processing
- Avoid using the same PIT timer on both cores
- Clear Dcache after writing to shared data arrays

### Memory and Performance
- Image processing optimized for 50Hz camera frame rate (20ms cycles)
- Use `float` for all floating-point calculations (Cortex-M7 FPU)
- Keep control loop deterministic within 1ms timing constraint

### Version Compatibility
- This project uses SeekFree library v3.9.1
- Update IAR project files when library version changes
- Motor mapping changed in version 5.12a (check README for details)

## Common Issues

### Build Issues
- Clean all temporary files before first build: run the batch file
- Check IAR project settings point to correct library paths
- Ensure both CM7_0 and CM7_1 projects build successfully

### Runtime Issues  
- Flight instability: Check IMU orientation mapping in `imu.h`
- Visual tracking errors: Verify camera calibration values
- Communication failures: Check `data_complex.h` UART configuration

### Configuration Drift
- Always reference `README.md` for latest coordinate system definitions
- Parameter changes should be documented in commit messages
- Test flight and debug modes after significant changes

This document reflects the current state as of commit 5.12a. Refer to README.md for complete version history and operational details.

---

# Karpathy Guidelines

Behavioral guidelines to reduce common LLM coding mistakes, derived from [Andrej Karpathy's observations](https://x.com/karpathy/status/2015883857489522876) on LLM coding pitfalls.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.
