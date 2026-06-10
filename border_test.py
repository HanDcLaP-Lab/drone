import numpy as np
import matplotlib.pyplot as plt
import math

# ==========================================
# 1. 宏定义与常量 (完全复刻 image.h 和 image.c)
# ==========================================
MT9V03X_W = 188
MT9V03X_H = 120

CAM_CX = 95.5754542
CAM_CY = 56.0345163

FOV_DIAMETER = 127.5
FOV_RADIUS = FOV_DIAMETER / 2.0
FOV_RADIUS_SQ = FOV_RADIUS * FOV_RADIUS

MORPH_MASK_DIAMETER = 100.0
MORPH_MASK_RADIUS = MORPH_MASK_DIAMETER / 2.0
MORPH_MASK_RADIUS_SQ = MORPH_MASK_RADIUS * MORPH_MASK_RADIUS

# ==========================================
# 2. 预计算数组分配
# ==========================================
fov_left_bound = np.zeros(MT9V03X_H, dtype=int)
fov_right_bound = np.zeros(MT9V03X_H, dtype=int)
morph_left_bound = np.zeros(MT9V03X_H, dtype=int)
morph_right_bound = np.zeros(MT9V03X_H, dtype=int)

# ==========================================
# 3. 视野边界生成逻辑 (复刻 camera_init)
# ==========================================
for r in range(MT9V03X_H):
    dy = r - CAM_CY
    dy_sq = dy * dy
    
    # FOV 有效边界
    if dy_sq > FOV_RADIUS_SQ:
        fov_left_bound[r] = MT9V03X_W
        fov_right_bound[r] = 0
    else:
        dx = math.sqrt(FOV_RADIUS_SQ - dy_sq)
        left = int(CAM_CX - dx)
        right = int(CAM_CX + dx)
        if left < 0: left = 0
        if right > MT9V03X_W: right = MT9V03X_W
        fov_left_bound[r] = left
        fov_right_bound[r] = right

    # Morph 保护遮罩边界
    if dy_sq > MORPH_MASK_RADIUS_SQ:
        morph_left_bound[r] = MT9V03X_W
        morph_right_bound[r] = 0
    else:
        dx = math.sqrt(MORPH_MASK_RADIUS_SQ - dy_sq)
        left = int(CAM_CX - dx)
        right = int(CAM_CX + dx)
        if left < 0: left = 0
        if right > MT9V03X_W: right = MT9V03X_W
        morph_left_bound[r] = left
        morph_right_bound[r] = right

# ==========================================
# 4. 边缘扫描点生成 (复刻修复后的 precompute_border_indices)
# ==========================================
border_indices = []

for r in range(1, MT9V03X_H - 1):
    c_start = fov_left_bound[r]
    c_end = fov_right_bound[r]
    
    if c_start >= c_end:
        continue # 该行完全在FOV外
        
    if c_start < 1:
        c_start = 1
    if c_end > MT9V03X_W - 1:
        c_end = MT9V03X_W - 1

    for c in range(c_start, c_end):
        is_edge = False
        
        # 上邻域在FOV外?
        if c < fov_left_bound[r - 1] or c >= fov_right_bound[r - 1]:
            is_edge = True
        # 下邻域在FOV外?
        elif c < fov_left_bound[r + 1] or c >= fov_right_bound[r + 1]:
            is_edge = True
        # 左邻域在FOV外? (即当前列是本行FOV的最左列)
        elif c == c_start:
            is_edge = True
        # 右邻域在FOV外? (即当前列是本行FOV的最右列)
        elif c + 1 >= c_end:
            is_edge = True
            
        if is_edge:
            border_indices.append((r, c))

print(f"生成的边界点数量(Border Pixel Count): {len(border_indices)}")

# ==========================================
# 5. 图像渲染与可视化
# ==========================================
# 创建一个纯黑色背景图像 (H, W, RGB)
img = np.zeros((MT9V03X_H, MT9V03X_W, 3), dtype=np.uint8)

# 逐像素上色
for r in range(MT9V03X_H):
    for c in range(MT9V03X_W):
        # 1. 画 FOV 白色有效区
        if c >= fov_left_bound[r] and c < fov_right_bound[r]:
            img[r, c] = [255, 255, 255] # 白色
            
        # 2. 画 形态学浅蓝色保护区
        if c >= morph_left_bound[r] and c < morph_right_bound[r]:
            img[r, c] = [173, 216, 230] # 浅蓝色 (Light Blue)

# 3. 画 红色边界种子点 (顶层覆盖)
for (r, c) in border_indices:
    img[r, c] = [255, 0, 0] # 红色

# 绘制与显示
plt.figure(figsize=(10, 6), dpi=120)
plt.imshow(img)

# 画个十字准星标出理论中心( CAM_CX, CAM_CY )
plt.scatter(CAM_CX, CAM_CY, color='magenta', marker='+', s=100, label='Camera Center')

plt.title('FOV Mask & Flood-Fill Seeds Visualization')
plt.xlabel('Columns (X)')
plt.ylabel('Rows (Y)')
plt.legend(loc='upper right')
plt.grid(color='gray', linestyle='--', linewidth=0.5, alpha=0.5)

# 反转Y轴使图像坐标系(0,0在左上角)与C语言数组一致
plt.gca().invert_yaxis() 
plt.show()
