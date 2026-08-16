# VL53L8CX Xtalk 校准工程

独立于原飞控工程，共享 `drone/libraries`，仅用于 VL53L8CX Xtalk 校准与导出。

## 目录

- `main.c`：校准主程序
- `driver/`：VL53L8CX ULD API + xtalk plugin + 平台层
- `iar/`：IAR 工程文件
- `icf/`：链接脚本

## 使用步骤

1. 用 IAR 打开 `iar/xtalk_calib.ewp`，编译下载。
2. 将 VL53L8CX 正对一个平坦目标，距离 **600mm**，目标尽量覆盖全视场（ST 推荐 3% 反射率目标）。
3. 打开串口，按提示发送任意字符开始校准。
4. 校准完成后，串口会打印 C 头文件格式数据。
5. 将打印内容保存为原工程头文件：
   `project/code/vl53l8cx_xtalk_calib_data.h`

## 原工程接入

1. 将导出的头文件保存为：
   `project/code/vl53l8cx_xtalk_calib_data.h`

2. 将 `project/code/tof.h` 中宏打开：
   `#define VL53L8CX_USE_XTALK_CALIB 1`

3. 重新编译原飞控工程即可。

`tof_init()` 中已加入条件加载：

```c
#if VL53L8CX_USE_XTALK_CALIB
    vl53l8cx_set_caldata_xtalk(&vl53l8cx_dev, (uint8_t*)vl53l8cx_xtalk_calib_data);
#endif
```

加载后启动测距即可使用校准后的 Xtalk 补偿。
