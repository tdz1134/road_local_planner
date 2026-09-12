#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_road_from_image.py —— 从图片生成闭合 loop 道路世界（v1.2）。

图片规范：
  · 格式：PNG 或 JPG
  · 道路图：黑色 = 道路，白色 = 背景
  · 障碍图（可选，--obstacles）：黑色 = 障碍，白色 = 背景
  · 或在道路图中用红色标记障碍
  · 分辨率：10 pixel/m（固定比例）
  · 坐标系：图片左上角 = 世界原点 (0,0)；x 向右增大，y 向下增大
    （因此世界坐标全部在第一象限，y 轴与 ROS 惯例相反）

用法：
    # 路缘模式（默认）
    python3 scripts/gen_road_from_image.py road.png

    # 墙壁模式（更干净，适合无障碍测试）
    python3 scripts/gen_road_from_image.py road.png --no-curb

    # 墙壁 + 自适应高度（路越宽墙越高）
    python3 scripts/gen_road_from_image.py road.png --no-curb --adaptive-wall

    # 带障碍（道路图中红色标记）
    python3 scripts/gen_road_from_image.py road.png --no-curb

    # 带障碍（单独障碍图）
    python3 scripts/gen_road_from_image.py road.png --no-curb --obstacles obs.png

参数调整：
    --simplify-tolerance 0.3   # 轮廓简化容差 m（越大越平滑，默认 0.3）
    --smooth-sigma 3.0         # 高斯平滑 σ 像素（越大越圆滑，默认 3.0）
    --wall-width 0.3           # 墙壁厚度 m（默认 0.3）
    --wall-height 1.5          # 墙壁高度 m（默认 1.5）
    --curb-height 1.5          # 路缘高度 m（默认 1.5）
    --obs-height 1.5           # 障碍高度 m（默认 1.5）
    --obs-min-area 0.5         # 最小障碍面积 m²（过滤噪点，默认 0.5）
"""

import argparse
import math
import os
import sys

try:
    import cv2
    import numpy as np
except ImportError:
    print("错误：需要 OpenCV 和 NumPy。安装：pip3 install opencv-python numpy",
          file=sys.stderr)
    sys.exit(1)


# ══════════════════ 常量 ══════════════════
PIXEL_PER_M = 10.0       # 图片分辨率：10 pixel/m

# 路缘模式默认参数
CURB_WIDTH = 0.3          # 路缘宽 m
CURB_HEIGHT = 1.5         # 路缘高 m
CURB_SEGMENT_LEN = 0.6    # 路缘段长 m
CURB_SPACING = 0.4        # 路缘段间距 m

# 墙壁模式默认参数
WALL_WIDTH = 0.3          # 墙壁厚 m
WALL_HEIGHT = 1.5         # 墙壁高 m
WALL_SEGMENT_LEN = 0.6    # 墙壁段长 m
WALL_SPACING = 0.4        # 墙壁段间距 m

# 障碍默认参数
OBS_HEIGHT = 1.5          # 障碍高 m

# 形态学核大小（像素）—— 消除像素级锯齿
MORPH_KERNEL_PX = 5

# 高斯平滑标准差（像素）—— 轮廓顶点平滑
GAUSS_SIGMA_PX = 3.0


# ══════════════════ 图片处理 ══════════════════

def load_binary(path):
    """读取图片 → 二值化（黑=道路=0，白=背景=255），含形态学闭运算平滑"""
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"无法读取图片：{path}")
    # 先二值化（红色障碍灰度≈76 < 128，会被标为道路 → 后续障碍检测从原彩色图做）
    _, binary = cv2.threshold(img, 128, 255, cv2.THRESH_BINARY)
    # 在二值图上做闭运算：填细小缺口、平滑边界锯齿
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                  (MORPH_KERNEL_PX, MORPH_KERNEL_PX))
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, k)
    return binary


def load_color(path):
    """读取彩色图片"""
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(f"无法读取图片：{path}")
    return img


# ══════════════════ 轮廓处理 ══════════════════

def extract_contours(binary):
    """提取轮廓（白色前景=对象）"""
    inverted = cv2.bitwise_not(binary)
    return cv2.findContours(inverted, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)


def smooth_contour(contour_px, sigma_px=GAUSS_SIGMA_PX):
    """高斯平滑闭合轮廓点（消除像素阶梯）"""
    pts = contour_px.reshape(-1, 2).astype(np.float64)
    n = len(pts)
    if n < 5:
        return pts
    ksize = max(3, int(sigma_px * 6) | 1)  # 确保奇数
    kernel = cv2.getGaussianKernel(ksize, sigma_px)
    # 对 x, y 分别做环形卷积（处理闭合轮廓的首尾衔接）
    out = np.empty_like(pts)
    for dim in range(2):
        col = pts[:, dim].reshape(-1, 1)
        # 首尾各延拓 ksize//2 个点（环形）
        half = ksize // 2
        ext = np.concatenate([col[-half:], col, col[:half]])
        filtered = cv2.filter2D(ext, -1, kernel).flatten()
        out[:, dim] = filtered[half:half + n]
    return out


def simplify_contour(contour_px, tolerance_m=0.3):
    """Ramer–Douglas–Peucker 简化。tolerance_m 单位米，输入/输出像素坐标"""
    eps_px = tolerance_m * PIXEL_PER_M
    result = cv2.approxPolyDP(contour_px.reshape(-1, 1, 2).astype(np.float32),
                              eps_px, closed=True)
    return result.reshape(-1, 2)


def contour_to_world(contour_px, img_h):
    """像素 → 世界坐标（翻转 y）"""
    return np.array([[p[0] / PIXEL_PER_M, (img_h - p[1]) / PIXEL_PER_M]
                     for p in contour_px])


# ══════════════════ 路缘 box ══════════════════

def gen_curb_boxes(wall_pts, width, height, seg_len, spacing):
    """沿轮廓生成路缘 box：[(cx, cy, yaw, L, W, H), ...]"""
    boxes = []
    n = len(wall_pts)
    if n < 2:
        return boxes
    seg_L = np.array([np.linalg.norm(wall_pts[(i+1) % n] - wall_pts[i])
                      for i in range(n)])
    cum = np.concatenate([[0.0], np.cumsum(seg_L)])
    total = cum[-1]
    s, idx = 0.0, 0
    while s < total:
        while idx < n and cum[idx + 1] <= s:
            idx += 1
        if idx >= n:
            break
        L = seg_L[idx]
        if L < 1e-6:
            s += spacing
            continue
        t = (s - cum[idx]) / L
        p1, p2 = wall_pts[idx], wall_pts[(idx + 1) % n]
        cx = p1[0] + t * (p2[0] - p1[0])
        cy = p1[1] + t * (p2[1] - p1[1])
        yaw = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        boxes.append((cx, cy, yaw, seg_len, width, height))
        s += spacing
    return boxes


# ══════════════════ 自适应路宽 ══════════════════

def compute_road_widths(outer_w, inner_w):
    """对每个外圈点，找内圈最近点距离 → 路宽（米）。返回数组"""
    from scipy.spatial import cKDTree
    tree = cKDTree(inner_w)
    dists, _ = tree.query(outer_w)
    return dists  # 每个外圈点对应的路宽（米）


def gen_adaptive_wall_boxes(wall_pts, road_widths, base_width, base_height,
                            seg_len, spacing, min_h=0.8, max_h=2.5):
    """沿轮廓生成墙壁 box，高度按路宽自适应缩放。
    路宽 = median 时高度 = base_height；路宽越窄墙越矮，越宽墙越高。
    """
    boxes = []
    n = len(wall_pts)
    if n < 2:
        return boxes
    base_ref = float(np.median(road_widths)) if len(road_widths) > 0 else 10.0
    seg_L = np.array([np.linalg.norm(wall_pts[(i+1) % n] - wall_pts[i])
                      for i in range(n)])
    cum = np.concatenate([[0.0], np.cumsum(seg_L)])
    total = cum[-1]
    # 路宽数组也按弧长参数化
    rw_cum = np.linspace(0, total, len(road_widths))

    s, idx = 0.0, 0
    while s < total:
        while idx < n and cum[idx + 1] <= s:
            idx += 1
        if idx >= n:
            break
        L = seg_L[idx]
        if L < 1e-6:
            s += spacing
            continue
        t = (s - cum[idx]) / L
        p1, p2 = wall_pts[idx], wall_pts[(idx + 1) % n]
        cx = p1[0] + t * (p2[0] - p1[0])
        cy = p1[1] + t * (p2[1] - p1[1])
        yaw = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        # 插值路宽
        rw = float(np.interp(s, rw_cum, road_widths))
        # 高度按路宽比例缩放
        h = base_height * (rw / base_ref) if base_ref > 0 else base_height
        h = max(min_h, min(max_h, h))
        boxes.append((cx, cy, yaw, seg_len, base_width, h))
        s += spacing
    return boxes


# ══════════════════ 障碍提取 ══════════════════

def extract_obstacles_from_red(img_color, img_h, min_area_m2=0.5,
                               obs_height=OBS_HEIGHT):
    """从彩色图中提取红色像素作为障碍 → 包围盒列表"""
    hsv = cv2.cvtColor(img_color, cv2.COLOR_BGR2HSV)
    # 红色在 HSV 色相环上有两段
    mask1 = cv2.inRange(hsv, (0, 80, 80), (10, 255, 255))
    mask2 = cv2.inRange(hsv, (170, 80, 80), (180, 255, 255))
    mask = cv2.bitwise_or(mask1, mask2)
    return _contours_to_obs_boxes(mask, img_h, min_area_m2, obs_height)


def extract_obstacles_from_image(path, img_h, min_area_m2=0.5,
                                 obs_height=OBS_HEIGHT):
    """从单独图片提取障碍（黑色=障碍，白色=背景）"""
    gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise FileNotFoundError(f"无法读取障碍图片：{path}")
    _, binary = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)
    # 反转：黑色障碍 → 白色前景（findContours 需要）
    inverted = cv2.bitwise_not(binary)
    return _contours_to_obs_boxes(inverted, img_h, min_area_m2, obs_height)


def _contours_to_obs_boxes(mask, img_h, min_area_m2, obs_height):
    """从二值 mask（白色=障碍）提取轮廓 → 用最小包围圆或旋转矩形近似"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    min_area_px = min_area_m2 * PIXEL_PER_M ** 2
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area_px:
            continue
        rect = cv2.minAreaRect(c)
        (rcx, rcy), (rw, rh), angle = rect
        # 长短边比
        long_side = max(rw, rh)
        short_side = min(rw, rh)
        aspect = short_side / long_side if long_side > 0 else 1.0
        if aspect > 0.85:
            # 近圆形 → 用短边做正方形 box（等效于包围圆）
            wx = rcx / PIXEL_PER_M
            wy = (img_h - rcy) / PIXEL_PER_M
            d = long_side / PIXEL_PER_M
            boxes.append((wx, wy, 0.0, d, d, obs_height))
        else:
            #  elongated → 旋转矩形
            wx = rcx / PIXEL_PER_M
            wy = (img_h - rcy) / PIXEL_PER_M
            ww = rw / PIXEL_PER_M
            wh = rh / PIXEL_PER_M
            yaw = math.radians(angle)
            boxes.append((wx, wy, yaw, ww, wh, obs_height))
    return boxes


# ══════════════════ SDF 输出 ══════════════════

def _box_sdf(name, cx, cy, cz, yaw, sx, sy, sz,
             ambient="0.6 0.6 0.6 1", diffuse="0.6 0.6 0.6 1"):
    """生成单个 box model 的 SDF 字符串"""
    return (
        f'    <model name="{name}">\n'
        f'      <static>true</static>\n'
        f'      <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 {yaw:.3f}</pose>\n'
        f'      <link name="link">\n'
        f'        <collision name="col">\n'
        f'          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>\n'
        f'        </collision>\n'
        f'        <visual name="vis">\n'
        f'          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>\n'
        f'          <material>\n'
        f'            <ambient>{ambient}</ambient>\n'
        f'            <diffuse>{diffuse}</diffuse>\n'
        f'          </material>\n'
        f'        </visual>\n'
        f'      </link>\n'
        f'    </model>\n'
    )


def write_world(boxes, output_path, world_name):
    """写 SDF world。boxes = [(name, cx, cy, yaw, L, W, H), ...]"""
    with open(output_path, 'w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<sdf version="1.5">\n')
        f.write(f'  <world name="{world_name}">\n')
        f.write('    <include><uri>model://sun</uri></include>\n')
        f.write('    <include><uri>model://ground_plane</uri></include>\n')
        for name, cx, cy, yaw, L, W, H in boxes:
            cz = H / 2.0
            if name.startswith("obs_"):
                amb = "0.8 0.3 0.0 1"
                dif = "0.8 0.3 0.0 1"
            else:
                amb = "0.6 0.6 0.6 1"
                dif = "0.6 0.6 0.6 1"
            f.write(_box_sdf(name, cx, cy, cz, yaw, L, W, H, amb, dif))
        f.write('  </world>\n')
        f.write('</sdf>\n')


# ══════════════════ 主流程 ══════════════════

def main():
    ap = argparse.ArgumentParser(
        description="从图片生成闭合 loop 道路世界（v1.1 改善平滑 + 障碍支持）")
    ap.add_argument("image", help="道路图片（黑=路，白=背景；红=障碍可选）")
    ap.add_argument("-o", "--output", help="输出 world 路径")
    ap.add_argument("--no-curb", action="store_true",
                    help="不生成路缘，改用薄墙标记边界")
    ap.add_argument("--obstacles",
                    help="障碍图片（黑=障碍，白=背景）；不指定则检测道路图中的红色")
    ap.add_argument("--simplify-tolerance", type=float, default=0.3,
                    help="轮廓简化容差 m（默认 0.3，越大越平滑）")
    ap.add_argument("--smooth-sigma", type=float, default=GAUSS_SIGMA_PX,
                    help=f"高斯平滑 σ 像素（默认 {GAUSS_SIGMA_PX}）")
    ap.add_argument("--wall-height", type=float, default=WALL_HEIGHT)
    ap.add_argument("--wall-width", type=float, default=WALL_WIDTH)
    ap.add_argument("--adaptive-wall", action="store_true",
                    help="墙壁高度按路宽自适应缩放（路越宽墙越高）")
    ap.add_argument("--curb-height", type=float, default=CURB_HEIGHT)
    ap.add_argument("--obs-height", type=float, default=OBS_HEIGHT)
    ap.add_argument("--obs-min-area", type=float, default=0.5,
                    help="最小障碍面积 m²（过滤噪点，默认 0.5）")
    args = ap.parse_args()

    if not os.path.exists(args.image):
        print(f"错误：图片不存在：{args.image}", file=sys.stderr)
        sys.exit(1)

    output = args.output or os.path.splitext(args.image)[0] + ".world"
    world_name = os.path.splitext(os.path.basename(output))[0]

    # ── 1. 读图 ──
    print(f"读取图片：{args.image}")
    binary = load_binary(args.image)
    img_h, img_w = binary.shape
    print(f"  {img_w}×{img_h} px → {img_w/PIXEL_PER_M:.1f}×{img_h/PIXEL_PER_M:.1f} m")

    # ── 2. 提取轮廓 ──
    print("提取轮廓...")
    contours, hierarchy = extract_contours(binary)
    print(f"  找到 {len(contours)} 个轮廓")
    if len(contours) < 2:
        print("错误：需要至少 2 个轮廓形成 loop 道路", file=sys.stderr)
        sys.exit(1)

    by_area = sorted(contours, key=cv2.contourArea, reverse=True)
    outer_px, inner_px = by_area[0], by_area[1]
    print(f"  外圈 {len(outer_px)} px，内圈 {len(inner_px)} px")

    # ── 3. 平滑 + 简化 ──
    print(f"平滑 σ={args.smooth_sigma:.1f}px，简化容差={args.simplify_tolerance}m")
    outer_smooth = smooth_contour(outer_px, args.smooth_sigma)
    inner_smooth = smooth_contour(inner_px, args.smooth_sigma)
    outer_s = simplify_contour(outer_smooth, args.simplify_tolerance)
    inner_s = simplify_contour(inner_smooth, args.simplify_tolerance)
    print(f"  外圈 {len(outer_s)} 点，内圈 {len(inner_s)} 点")

    outer_w = contour_to_world(outer_s, img_h)
    inner_w = contour_to_world(inner_s, img_h)

    # ── 4. 生成边界 ──
    all_boxes = []  # [(name, cx, cy, yaw, L, W, H)]

    if args.no_curb:
        # 墙壁模式
        if args.adaptive_wall:
            # 自适应高度：检测路宽，按路宽比例缩放墙高
            print("  检测路宽（自适应模式）...")
            rw_outer = compute_road_widths(outer_w, inner_w)
            rw_inner = compute_road_widths(inner_w, outer_w)
            wall_boxes_l = gen_adaptive_wall_boxes(
                outer_w, rw_outer, args.wall_width, args.wall_height,
                WALL_SEGMENT_LEN, WALL_SPACING)
            wall_boxes_r = gen_adaptive_wall_boxes(
                inner_w, rw_inner, args.wall_width, args.wall_height,
                WALL_SEGMENT_LEN, WALL_SPACING)
            print(f"  路宽范围：{min(rw_outer):.1f}~{max(rw_outer):.1f}m "
                  f"（中位 {np.median(rw_outer):.1f}m）")
        else:
            wall_boxes_l = gen_curb_boxes(outer_w, args.wall_width,
                                          args.wall_height,
                                          WALL_SEGMENT_LEN, WALL_SPACING)
            wall_boxes_r = gen_curb_boxes(inner_w, args.wall_width,
                                          args.wall_height,
                                          WALL_SEGMENT_LEN, WALL_SPACING)
        for i, b in enumerate(wall_boxes_l):
            all_boxes.append((f"wallL_{i}", *b))
        for i, b in enumerate(wall_boxes_r):
            all_boxes.append((f"wallR_{i}", *b))
        print(f"  墙壁：外 {len(wall_boxes_l)} + 内 {len(wall_boxes_r)}")
    else:
        # 路缘模式
        curb_l = gen_curb_boxes(outer_w, CURB_WIDTH, args.curb_height,
                                CURB_SEGMENT_LEN, CURB_SPACING)
        curb_r = gen_curb_boxes(inner_w, CURB_WIDTH, args.curb_height,
                                CURB_SEGMENT_LEN, CURB_SPACING)
        for i, b in enumerate(curb_l):
            all_boxes.append((f"curbL_{i}", *b))
        for i, b in enumerate(curb_r):
            all_boxes.append((f"curbR_{i}", *b))
        print(f"  路缘：外 {len(curb_l)} + 内 {len(curb_r)}")

    # ── 5. 障碍 ──
    obs_boxes = []
    if args.obstacles:
        obs_boxes = extract_obstacles_from_image(
            args.obstacles, img_h, args.obs_min_area, args.obs_height)
        print(f"  障碍（从图片）：{len(obs_boxes)} 个")
    else:
        # 尝试从道路图检测红色
        color = load_color(args.image)
        obs_boxes = extract_obstacles_from_red(
            color, img_h, args.obs_min_area, args.obs_height)
        if obs_boxes:
            print(f"  障碍（从红色）：{len(obs_boxes)} 个")

    for i, b in enumerate(obs_boxes):
        all_boxes.append((f"obs_{i}", *b))

    # ── 6. 输出 ──
    print(f"输出：{output}（{len(all_boxes)} 个 model）")
    write_world(all_boxes, output, world_name)
    print("完成")


if __name__ == "__main__":
    main()
