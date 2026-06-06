import math
import numpy as np
import matplotlib.pyplot as plt
import os

MT9V03X_H = 120
MT9V03X_W = 188
IMG_CENTER_X = 94.0
IMG_CENTER_Y = 60.0
FOV_DIAMETER = 126.0
FOV_RADIUS = FOV_DIAMETER / 2.0
FOV_RADIUS_SQ = FOV_RADIUS * FOV_RADIUS

CX = 95.5754542924
CY = 56.0345163934
A0 = 53.5431129928
A2 = -0.0157824974
A3 = 0.0002786109
A4 = -0.0000044018

def get_dist(c, r, height=110.0):
    u_prime = c - CX
    v_prime = r - CY
    x = u_prime
    y = -v_prime
    rho2 = x*x + y*y
    rho = math.sqrt(rho2)
    z_poly = A0 + A2 * rho2 + A3 * rho2 * rho + A4 * rho2 * rho2
    
    ray_z = -z_poly
    body_z = ray_z
    
    if body_z > -0.01:
        body_z = -0.01
        
    scale = -height / body_z
    px = y * scale
    py = x * scale
    return math.sqrt(px*px + py*py)

def main():
    fov_left_bound = [0] * MT9V03X_H
    fov_right_bound = [0] * MT9V03X_H
    
    for r in range(MT9V03X_H):
        dy = r - IMG_CENTER_Y
        dy_sq = dy * dy
        if dy_sq > FOV_RADIUS_SQ:
            fov_left_bound[r] = MT9V03X_W
            fov_right_bound[r] = 0
        else:
            dx = math.sqrt(FOV_RADIUS_SQ - dy_sq)
            fov_left_bound[r] = int(IMG_CENTER_X - dx)
            fov_right_bound[r] = int(IMG_CENTER_X + dx)

    safe_margin = 1
    edge_points_all = []
    
    img = np.zeros((MT9V03X_H, MT9V03X_W, 3), dtype=np.uint8)
    
    # 填充有效 FOV
    for r in range(safe_margin, MT9V03X_H - safe_margin):
        bc_start = max(fov_left_bound[r], safe_margin)
        bc_end = min(fov_right_bound[r], MT9V03X_W - safe_margin)
        for c in range(bc_start, bc_end):
            img[r, c] = [30, 80, 30]
            
    # 第一遍寻找所有物理边缘点
    for r in range(safe_margin, MT9V03X_H - safe_margin):
        bc_start = max(fov_left_bound[r], safe_margin)
        bc_end = min(fov_right_bound[r], MT9V03X_W - safe_margin)
        
        for c in range(bc_start, bc_end):
            is_edge = False
            if r - 1 < safe_margin or c < fov_left_bound[r - 1] or c >= fov_right_bound[r - 1]:
                is_edge = True
            elif r + 1 >= MT9V03X_H - safe_margin or c < fov_left_bound[r + 1] or c >= fov_right_bound[r + 1]:
                is_edge = True
            elif c - 1 < bc_start:
                is_edge = True
            elif c + 1 >= bc_end:
                is_edge = True
                
            if is_edge:
                edge_points_all.append((c, r))

    # 1. 大于10米的采样
    dist_filtered = []
    for (c, r) in edge_points_all:
        if get_dist(c, r) > 1000.0:
            dist_filtered.append((c, r))
            
    sampled = dist_filtered[::2]
    sampled_set = set(sampled)
    
    # 2. 针对右侧区域没有点的问题，强制在右侧最边缘补充点
    # 由于CX=95.5偏右，导致右边缘物理距离不足10m被筛掉
    # 我们找出所有位于右边缘（例如 c > 130）且未在 sampled 中的点
    right_points = []
    for (c, r) in edge_points_all:
        if c > 130 and (c, r) not in sampled_set:
            right_points.append((c, r))
            
    # 从中均匀抽取约 20 个点
    if len(right_points) > 0:
        step = max(1, len(right_points) // 20)
        extra_points = right_points[::step][:20]
        for p in extra_points:
            if p not in sampled_set:
                sampled.append(p)
                sampled_set.add(p)

    # 为了保持数组里的 index 从小到大排序（利于缓存命中），按 (r, c) 排序
    sampled.sort(key=lambda p: (p[1], p[0]))

    # 绘制红色点
    for (c, r) in sampled:
        for dr in [-1, 0, 1]:
            for dc in [-1, 0, 1]:
                nr = r + dr
                nc = c + dc
                if 0 <= nr < MT9V03X_H and 0 <= nc < MT9V03X_W:
                    img[nr, nc] = [255, 50, 50]

    # 画十字光心
    ccx, ccy = int(CX), int(CY)
    for i in range(-3, 4):
        if 0 <= ccy+i < MT9V03X_H: img[ccy+i, ccx] = [255, 255, 0]
        if 0 <= ccx+i < MT9V03X_W: img[ccy, ccx+i] = [255, 255, 0]
        
    plt.figure(figsize=(10, 6), facecolor='white')
    plt.imshow(img)
    plt.title(f"Effective FOV & Sampled Edge Points (Total: {len(sampled)})", pad=15)
    plt.axis('off')
    
    artifact_path = r"C:\Users\26784\.gemini\antigravity\brain\da14e3e0-1f98-4f56-85b5-deb3ace2f0ad\fov_distribution.png"
    plt.savefig(artifact_path, bbox_inches='tight', dpi=150)
    print(f"Image saved to {artifact_path}")
    
    # 打印 C 数组
    indices = [r * MT9V03X_W + c for (c, r) in sampled]
    print("const uint16_t precomputed_border_indices[] = {")
    chunk_size = 10
    for i in range(0, len(indices), chunk_size):
        chunk = indices[i:i+chunk_size]
        print("    " + ", ".join(str(x) for x in chunk) + ",")
    print("};")
    print(f"const int precomputed_border_count = {len(indices)};")

if __name__ == "__main__":
    main()
