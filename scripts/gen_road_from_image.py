#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_road_from_image.py —— 从图片生成闭合 loop 道路世界。

图片规范 v1.0：
  · 格式：PNG 或 JPG
  · 颜色：黑色 (0,0,0) = 道路，白色 (255,255,255) = 背景
  · 分辨率：10 pixel/m（固定比例）
  · 坐标系：图片左上角 = 世界原点 (0,0)，y 轴向下
  · 道路：黑色区域形成闭合环（内外两圈轮廓）

用法：
    python3 scripts/gen_road_from_image.py road.png
    python3 scripts/gen_road_from_image.py road.png -o output.world
    python3 scripts/gen_road_from_image.py road.png --curb-height 0.5  # 矮路缘
"""

import argparse
import math
import os
import sys

try:
    import cv2
    import numpy as np
except ImportError:
    print("错误：需要 OpenCV 和 NumPy。安装：pip3 install opencv-python numpy", file=sys.stderr)
    sys.exit(1)


# ══════════════════ 与 gen_road_world.py 保持同步 ══════════════════
PIXEL_PER_M = 10.0       # 图片分辨率：10 pixel/m
CURB_WIDTH = 0.3         # 路缘宽 m
CURB_HEIGHT = 1.5        # 路缘高 m
CURB_SEGMENT_LEN = 0.6   # 路缘段长 m（沿轮廓方向）
CURB_SPACING = 0.4       # 路缘段间距 m（中心到中心）
# ═══════════════════════════════════════════════════════════════════


def load_image(path):
    """读取图片并二值化（黑色 = 道路 = 0，白色 = 背景 = 255）"""
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"无法读取图片：{path}")
    # 二值化：黑色 (< 128) = 道路 = 0，白色 (>= 128) = 背景 = 255
    _, binary = cv2.threshold(img, 128, 255, cv2.THRESH_BINARY)
    return binary


def extract_contours(binary):
    """提取轮廓，返回轮廓列表（每个轮廓是 Nx1x2 的 ndarray）"""
    # findContours 需要白色前景（255）= 对象，黑色 (0) = 背景
    # 我们的 binary 里黑色 = 道路，所以需要反转
    inverted = cv2.bitwise_not(binary)
    contours, hierarchy = cv2.findContours(inverted, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    return contours, hierarchy


def simplify_contour(contour, tolerance=0.2):
    """Ramer–Douglas–Peucker 简化轮廓，tolerance 单位是米，返回像素坐标"""
    epsilon_px = tolerance * PIXEL_PER_M
    simplified = cv2.approxPolyDP(contour, epsilon_px, closed=True)
    return simplified.reshape(-1, 2)  # 返回像素坐标


def pixel_to_world(pt, image_height):
    """像素坐标 → 世界坐标（翻转 y 轴）"""
    return np.array([pt[0] / PIXEL_PER_M, (image_height - pt[1]) / PIXEL_PER_M])


def contour_to_world(contour_px, image_height):
    """轮廓点（像素）→ 世界坐标（米）"""
    world = np.array([[pt[0] / PIXEL_PER_M, (image_height - pt[1]) / PIXEL_PER_M]
                      for pt in contour_px])
    return world


def generate_curb_boxes(contour_world, curb_width, curb_height, segment_len, spacing):
    """沿轮廓生成路缘 box，返回 box 列表 [(cx, cy, yaw, length, width, height), ...]"""
    boxes = []
    n = len(contour_world)
    if n < 2:
        return boxes

    # 计算每段长度和累计弧长
    seg_lengths = []
    cum_lengths = [0.0]  # cum_lengths[i] = 前 i 段的累计长度
    for i in range(n):
        p1 = contour_world[i]
        p2 = contour_world[(i + 1) % n]
        L = np.linalg.norm(p2 - p1)
        seg_lengths.append(L)
        cum_lengths.append(cum_lengths[-1] + L)
    total_len = cum_lengths[-1]

    # 沿轮廓按 spacing 均匀采样（双指针法）
    s = 0.0
    seg_idx = 0
    while s < total_len:
        # 找到 s 所在的段（cum_lengths[seg_idx] <= s < cum_lengths[seg_idx+1]）
        while seg_idx < n and cum_lengths[seg_idx + 1] <= s:
            seg_idx += 1
        if seg_idx >= n:
            break

        p1 = contour_world[seg_idx]
        p2 = contour_world[(seg_idx + 1) % n]
        seg_len = seg_lengths[seg_idx]
        if seg_len < 1e-6:
            s += spacing
            continue

        # 在段内插值
        t = (s - cum_lengths[seg_idx]) / seg_len
        cx = p1[0] + t * (p2[0] - p1[0])
        cy = p1[1] + t * (p2[1] - p1[1])
        yaw = math.atan2(p2[1] - p1[1], p2[0] - p1[0])

        boxes.append((cx, cy, yaw, segment_len, curb_width, curb_height))
        s += spacing

    return boxes


def write_sdf_world(boxes_left, boxes_right, output_path, world_name="road_from_image"):
    """输出 SDF world 文件"""
    with open(output_path, 'w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<sdf version="1.5">\n')
        f.write('  <world name="%s">\n' % world_name)
        f.write('    <include><uri>model://sun</uri></include>\n')
        f.write('    <include><uri>model://ground_plane</uri></include>\n')

        model_id = 0
        for cx, cy, yaw, length, width, height in boxes_left:
            f.write(f'    <model name="curbL_{model_id}">\n')
            f.write(f'      <static>true</static>\n')
            f.write(f'      <pose>{cx:.3f} {cy:.3f} {height/2:.3f} 0 0 {yaw:.3f}</pose>\n')
            f.write(f'      <link name="link">\n')
            f.write(f'        <collision name="collision">\n')
            f.write(f'          <geometry><box><size>{length:.3f} {width:.3f} {height:.3f}</size></box></geometry>\n')
            f.write(f'        </collision>\n')
            f.write(f'        <visual name="visual">\n')
            f.write(f'          <geometry><box><size>{length:.3f} {width:.3f} {height:.3f}</size></box></geometry>\n')
            f.write(f'          <material>\n')
            f.write(f'            <ambient>0.6 0.6 0.6 1</ambient>\n')
            f.write(f'            <diffuse>0.6 0.6 0.6 1</diffuse>\n')
            f.write(f'          </material>\n')
            f.write(f'        </visual>\n')
            f.write(f'      </link>\n')
            f.write(f'    </model>\n')
            model_id += 1

        model_id = 0
        for cx, cy, yaw, length, width, height in boxes_right:
            f.write(f'    <model name="curbR_{model_id}">\n')
            f.write(f'      <static>true</static>\n')
            f.write(f'      <pose>{cx:.3f} {cy:.3f} {height/2:.3f} 0 0 {yaw:.3f}</pose>\n')
            f.write(f'      <link name="link">\n')
            f.write(f'        <collision name="collision">\n')
            f.write(f'          <geometry><box><size>{length:.3f} {width:.3f} {height:.3f}</size></box></geometry>\n')
            f.write(f'        </collision>\n')
            f.write(f'        <visual name="visual">\n')
            f.write(f'          <geometry><box><size>{length:.3f} {width:.3f} {height:.3f}</size></box></geometry>\n')
            f.write(f'          <material>\n')
            f.write(f'            <ambient>0.6 0.6 0.6 1</ambient>\n')
            f.write(f'            <diffuse>0.6 0.6 0.6 1</diffuse>\n')
            f.write(f'          </material>\n')
            f.write(f'        </visual>\n')
            f.write(f'      </link>\n')
            f.write(f'    </model>\n')
            model_id += 1

        f.write('  </world>\n')
        f.write('</sdf>\n')


def main():
    parser = argparse.ArgumentParser(description="从图片生成闭合 loop 道路世界")
    parser.add_argument("image", help="输入图片路径（PNG/JPG）")
    parser.add_argument("-o", "--output", help="输出 world 路径（默认：图片名.world）")
    parser.add_argument("--curb-height", type=float, default=CURB_HEIGHT, help=f"路缘高度 m（默认 {CURB_HEIGHT}）")
    parser.add_argument("--simplify-tolerance", type=float, default=0.2, help=f"轮廓简化容差 m（默认 0.2）")
    args = parser.parse_args()

    if not os.path.exists(args.image):
        print(f"错误：图片不存在：{args.image}", file=sys.stderr)
        sys.exit(1)

    output = args.output or os.path.splitext(args.image)[0] + ".world"

    print(f"读取图片：{args.image}")
    binary = load_image(args.image)
    h, w = binary.shape
    print(f"  尺寸：{w}×{h} pixel → {w/PIXEL_PER_M:.1f}×{h/PIXEL_PER_M:.1f} m")

    print("提取轮廓...")
    contours, hierarchy = extract_contours(binary)
    print(f"  找到 {len(contours)} 个轮廓")

    if len(contours) < 2:
        print("错误：需要至少 2 个轮廓（外圈和内圈）才能形成 loop 道路", file=sys.stderr)
        sys.exit(1)

    # 按面积排序，取最大的两个（外圈和内圈）
    contours_sorted = sorted(contours, key=cv2.contourArea, reverse=True)
    outer_px = contours_sorted[0]
    inner_px = contours_sorted[1]

    print(f"  外圈：{len(outer_px)} 点，面积 {cv2.contourArea(outer_px)/PIXEL_PER_M**2:.1f} m²")
    print(f"  内圈：{len(inner_px)} 点，面积 {cv2.contourArea(inner_px)/PIXEL_PER_M**2:.1f} m²")

    print("简化轮廓...")
    # 用很小的容差（0.05m）简化，保留大部分细节
    outer_simplified = simplify_contour(outer_px, min(args.simplify_tolerance, 0.05))
    inner_simplified = simplify_contour(inner_px, min(args.simplify_tolerance, 0.05))
    print(f"  外圈：{len(outer_simplified)} 点")
    print(f"  内圈：{len(inner_simplified)} 点")

    print("转换坐标...")
    outer_world = contour_to_world(outer_simplified, h)
    inner_world = contour_to_world(inner_simplified, h)

    print("生成路缘 box...")
    boxes_left = generate_curb_boxes(outer_world, CURB_WIDTH, args.curb_height,
                                      CURB_SEGMENT_LEN, CURB_SPACING)
    boxes_right = generate_curb_boxes(inner_world, CURB_WIDTH, args.curb_height,
                                       CURB_SEGMENT_LEN, CURB_SPACING)
    print(f"  左路缘：{len(boxes_left)} 个 box")
    print(f"  右路缘：{len(boxes_right)} 个 box")

    print(f"输出 world：{output}")
    world_name = os.path.splitext(os.path.basename(output))[0]
    write_sdf_world(boxes_left, boxes_right, output, world_name)
    print("完成")


if __name__ == "__main__":
    main()
