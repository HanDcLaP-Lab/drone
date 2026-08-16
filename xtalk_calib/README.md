# VL53L8CX Xtalk 校准工程

基于 SeekFree CYT4BB 空工程，仅 CM7_0 编写校准代码，CM7_1 保持空工程不变。

## 目录

- user/main_cm7_0.c：校准主程序（仅 0 核）
- code/vl53l8cx/：VL53L8CX ULD API + xtalk plugin + 平台层
- iar/：IAR 工程
- icf/：链接脚本

## 使用

1. 用 IAR 打开 iar/xtalk_calib.eww。
2. 编译并下载 CM7_0（CM7_1 可保持空工程不下载/不修改）。
3. 将 VL53L8CX 正对平坦目标，距离 600mm，覆盖全视场。
4. 串口发送任意字符开始校准。
5. 校准完成后串口打印 C 头文件格式数据。

## 导出到原工程

1. 将打印内容保存为 project/code/vl53l8cx_xtalk_calib_data.h。
2. 原工程 project/code/tof.h 中打开：
   #define VL53L8CX_USE_XTALK_CALIB 1
3. 重新编译原飞控工程即可。