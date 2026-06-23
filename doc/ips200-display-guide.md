# IPS200 屏幕操作参考 (for AI)

> 本文档供 Claude Code 等 AI 工具在编写屏幕相关代码时参考。记录了硬件参数、API 语义、惯用法和常见坑。

---

## 硬件规格

| 属性 | 值 |
|------|-----|
| 分辨率 | 320 × 240 (物理像素) |
| 接口 | SPI (硬件 SPI_1, 30MHz) |
| 引脚 | SCL=P12_2, MOSI=P12_1, RST=P22_4, DC=P22_3, CS=P12_3, BL=P11_0 |
| 颜色格式 | RGB565 (uint16) |
| 默认字体 | 8×16 像素 (`IPS200_8X16_FONT`) |
| 6×8 字体 | 6×8 像素 (`IPS200_6X8_FONT`) |
| 16×16 字体 | 不支持 |

### 竖屏模式 (项目默认 `IPS200_PORTAIT`)

```
宽: 240 px   高: 320 px
8×16 字体 → 30列 × 20行
6×8 字体  → 40列 × 40行
```

### 横屏模式 (`IPS200_CROSSWISE`)

```
宽: 320 px   高: 240 px
8×16 字体 → 40列 × 15行
```

---

## 初始化模板

最小初始化（项目惯用）：

```c
#include "zf_common_headfile.h"

void display_init(void) {
    ips200_set_dir(IPS200_PORTAIT);       // 竖屏
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);  // 白字黑底
    ips200_init(IPS200_TYPE_SPI);         // SPI 模式
}
```

`ips200_init()` 内部已完成 GPIO、SPI、控制器寄存器序列、背景清屏，**无需额外操作**。

---

## 颜色枚举 (rgb565_color_enum)

定义在 `libraries/zf_common/zf_common_font.h:44-55`：

| 枚举名 | 十六进制 | 颜色 |
|--------|---------|------|
| `RGB565_WHITE` | `0xFFFF` | 白 |
| `RGB565_BLACK` | `0x0000` | 黑 |
| `RGB565_RED` | `0xF800` | 红 |
| `RGB565_GREEN` | `0x07E0` | 绿 |
| `RGB565_BLUE` | `0x001F` | 蓝 |
| `RGB565_YELLOW` | `0xFFE0` | 黄 |
| `RGB565_CYAN` | `0x07FF` | 青 |
| `RGB565_MAGENTA` / `RGB565_PURPLE` | `0xF81F` | 紫/品红 |
| `RGB565_GRAY` | `0x8430` | 灰 |
| `RGB565_BROWN` | `0xBC40` | 棕 |
| `RGB565_PINK` | `0xFE19` | 粉 |

---

## API 参考

### 全局状态

```c
ips200_set_dir(ips200_dir_enum dir);   // 设置方向，同时更新 ips200_width_max/height_max
ips200_set_color(uint16 pen, uint16 bgcolor);  // 设置前景色/背景色 (影响文字渲染)
ips200_set_font(ips200_font_size_enum font);   // 设置字体 (6x8 或 8x16)
```

### 全屏操作

```c
ips200_clear(void);                    // 用当前背景色填充全屏 ~42ms@30MHz
ips200_full(uint16 color);             // 用指定颜色填充全屏
```

### 像素/线

```c
ips200_draw_point(uint16 x, uint16 y, uint16 color);
ips200_draw_line(uint16 x1, uint16 y1, uint16 x2, uint16 y2, uint16 color);
```

坐标原点在**左上角**，X 向右，Y 向下。

### 文字显示

所有文字函数使用**像素坐标** (不是行列)，Y 坐标通常用 `16*n` 对齐行高。

```c
ips200_show_char(uint16 x, uint16 y, char dat);
ips200_show_string(uint16 x, uint16 y, const char str[]);  // 遇 '\0' 停止
ips200_show_int(uint16 x, uint16 y, int32 dat, uint8 num);  // 固定宽度，右对齐
ips200_show_uint(uint16 x, uint16 y, uint32 dat, uint8 num);
ips200_show_float(uint16 x, uint16 y, double dat, uint8 num, uint8 pointnum);
```

- `num`: 总宽度（字符数），**不包含**小数点。`ips200_show_float(x,y, 3.14, 4, 2)` → `3.14` (整数部分 4 位 + 小数点 + 小数 2 位)
- 中文用 `ips200_show_chinese(x, y, 16, chinese_buf, count, color)`——需要预制的取模数据

### 图像

```c
ips200_show_binary_image(x, y, image, raw_w, raw_h, disp_w, disp_h);
ips200_show_gray_image(x, y, image, raw_w, raw_h, disp_w, disp_h, threshold);
ips200_show_rgb565_image(x, y, image, raw_w, raw_h, disp_w, disp_h, color_mode);
```

项目惯用宏（MT9V03X 相机灰度图显示）：
```c
#define ips200_displayimage03x(p, width, height) \
    (ips200_show_gray_image(0, 0, (p), MT9V03X_W, MT9V03X_H, (width), (height), 0))
```

---

## ⚠️ 关键注意事项

### 1. 文字没有背景擦除 — 必须手动 clean 或有自绘背景

`ips200_show_char` / `ips200_show_string` **只画前景像素，不擦背景**。更新文字时旧内容不会自动消失，会叠加成鬼影。解决方案：

- **方案A**：每次刷新前 `ips200_clear()` 全屏擦除 → 简单但 ~42ms，适合 ≤1Hz 刷新
- **方案B**：先画背景色矩形再写文字（本驱动无 fill_rect 函数，需手动 `for` 循环像素写）
- **方案C**：先 `ips200_set_color` 为背景色写一遍旧文字"擦除"，再换前景色写新文字（推荐 5-10Hz 局部更新时使用）

### 2. `ips200_clear()` 耗时 42ms

全屏擦除走 SPI 传输 240×320×2 = 153KB，@30MHz 约 42ms。**不能在 50Hz 循环里每帧调用**。控制刷新率 ≤2Hz。

### 3. 坐标边界

竖屏模式 `IPS200_PORTAIT` 下：
- X 范围 [0, 239]
- Y 范围 [0, 319]
- 每行最多 30 个 8×16 字符

越界写入会导致花屏或死机（驱动无边界检查）。

### 4. 文字函数 x 参数是像素坐标

```c
ips200_show_string(8, 16*3, "hello");   // 第 3 行，第 2 列开始
ips200_show_string(0, 16*0, "title");   // 第 0 行，第 0 列开始
```

列号转像素：`x = col * 8`（8×16 字体）；`x = col * 6`（6×8 字体）。

### 5. SPI 引脚已占用

IPS200 使用 `SPI_1`（P12_2 CLK, P12_1 MOSI, P12_3 CS），其他 SPI 设备不能共用这些引脚。

---

## 项目中的惯用法

### 显示初始化

```c
// main() 中，一次性调用
ips200_set_dir(IPS200_PORTAIT);
ips200_init(IPS200_TYPE_SPI);
```

### 表格数据布局

```c
// 标题
ips200_show_string(48, 0, "My Data Table");
ips200_draw_line(4, 18, 236, 18, RGB565_WHITE);  // 分隔线

// 列头
ips200_show_string(8, 22, "Col0  Col1  Col2  Col3");

// 数据行
ips200_show_string(8, 40 + r * 16, line);  // 行间距 = 字体高度 = 16

// 底部状态行
snprintf(buf, sizeof(buf), "status: %d", val);
ips200_show_string(0, 16 * 18, buf);  // 倒数第二行
```

### 清屏 + 重刷周期

```c
if (frame_cnt % 50 == 0) {   // ≤2Hz, 避免闪烁
    ips200_set_color(pen, bg);
    ips200_clear();
    draw_all_content();
}
```

### X/Y 值对：左列 x=0, 右列 x=120

项目惯用左右两列布局：
```c
ips200_show_string(0,   row, "label:");
ips200_show_int    (40,  row,  val, 4);
ips200_show_string(120, row, "label:");
ips200_show_int    (160, row,  val, 4);
```
