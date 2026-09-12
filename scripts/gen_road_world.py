#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_road_world.py —— 参数化生成「首尾闭合的纯道路」Gazebo 世界（SDF）。

背景：SDF 原生只有 box/cylinder/sphere/plane/mesh，没有「圆弧/曲线」碰撞体。
所以道路（含弯道）必须用「密集小直线 box 段」逼近曲率。本脚本用航向积分
（heading integration）参数化定义道路中心线，再两侧各偏移 ±路宽/2 生成路缘
墙段，输出一份完整 .world。改形状 = 改下面 PRESETS 里对应 preset 的
width / sections / ds 参数，重跑即可（--preset loop|extreme）。

────────────────────────────────────────────────────────────────────────
道路 = 两侧路缘墙夹出的自由走廊（与 road_world.world 的「路线 A」一致）：
  · 激光打到路缘 → grid_node 生成 free 带 = 道路；沿路模式据此跟路。
  · 路缘全为凸形小 box 段，接缝靠 JOINT_EXT 延伸压合，防激光漏过缝隙。
  · 沥青条 visual-only、贴地 z≈0.005 < grid_node z_min=0.1，激光打不到 → 不污染栅格。
  · 纯道路无障碍：走廊内不放任何圆柱。
────────────────────────────────────────────────────────────────────────

坐标约定：pose = x y z roll pitch yaw；box 的 <size>=长(x) 宽(y) 高(z)。
用法：
    python3 gen_road_world.py              # → 生成所有预设
    python3 gen_road_world.py -p loop      # → 仅 loop_road.world
    python3 gen_road_world.py -p extreme   # → 仅 loop_extreme.world
（输出文件名规则：loop_<tag>.world；preset=loop 时 tag=road 保持历史兼容）
"""

import math
import os
import argparse
import sys

# ══════════════════════ 可调参数（改这里即可）══════════════════════════
CURB_T = 0.3            # 路缘墙厚 m
CURB_H = 1.5            # 路缘墙高 m（须 > grid_node z_min，激光才打得到；沿用现有 1.5）
JOINT_EXT = 0.18        # 每段两端各延伸 m，压住相邻段接缝（防激光从缝里漏过去）
GROUND_SIZE = 120.0     # 地面尺寸 m（要罩住整条环路）
# ═══════════════════════════════════════════════════════════════════════

# 预设场景：width=路宽 ds=弧段离散步长 sections=中心线段序列 spawn=出生点(x,y,yaw)
# 段序列：('line', 长度) 或 ('arc', 半径, 圆心角deg)。正圆心角=左转，负=右转。
# 闭合条件：前半序列净转角 = 180°，后半重复同一序列 → 点对称自动闭合。
PRESETS = {
    # 常规闭合环：直道 + 小曲率缓弯 + 适中 U 形弯，路宽 6m（inflation 0.7 → 净通行 4.6m）
    'loop': {
        'width': 6.0, 'ds': 0.40, 'spawn': (2.0, 0.0, 0.0),
        'sections': [
            ('line', 18.0),
            ('arc', 12.0, 45.0),    # 小曲率缓弯
            ('line', 5.0),
            ('arc', 5.0, 135.0),    # 适中 U 形弯（内侧路缘半径 = 5 - 3 = 2m）
            ('line', 18.0),
            ('arc', 12.0, 45.0),
            ('line', 5.0),
            ('arc', 5.0, 135.0),
        ],
    },
    # 极端闭合环：路宽压到 3m（净通行 1.6m ≈ 车宽 0.92 + 每侧 0.34），
    # S 形 chicane（R3 左右背靠背 90°）+ 180° 发夹弯（R=2.5，内侧路缘半径 0.85m），
    # 发夹处前瞻 7m 全程撞墙，扇形打分退化为纯 align —— 专挑跟路/绕障的临界点。
    'extreme': {
        'width': 3.0, 'ds': 0.25, 'spawn': (2.0, 0.0, 0.0),
        'sections': [
            ('line', 15.0),
            ('arc', 3.0, 90.0),     # S-chicane：左 90°
            ('arc', 3.0, -90.0),    # S-chicane：右 90°（背靠背反弯）
            ('line', 4.0),          # 短直道，无回正缓冲直接进发夹
            ('arc', 2.5, 180.0),    # 180° 发夹（近乎原地掉头）
            ('line', 15.0),
            ('arc', 3.0, 90.0),
            ('arc', 3.0, -90.0),
            ('line', 4.0),
            ('arc', 2.5, 180.0),
        ],
    },
}


# ── SDF 片段生成 ──────────────────────────────────────────────────────

def box_model(name, cx, cy, yaw, length, width, height, z_center,
              collision, color):
    """生成一个静态 box model 的 SDF 片段；collision=False 时仅 visual。"""
    col = ""
    if collision:
        col = (
            f"        <collision name=\"collision\">\n"
            f"          <geometry><box>"
            f"<size>{length:.3f} {width:.3f} {height:.3f}</size>"
            f"</box></geometry>\n"
            f"        </collision>\n"
        )
    return (
        f"    <model name=\"{name}\">\n"
        f"      <static>true</static>\n"
        f"      <pose>{cx:.3f} {cy:.3f} {z_center:.3f} 0 0 {yaw:.5f}</pose>\n"
        f"      <link name=\"link\">\n"
        f"{col}"
        f"        <visual name=\"visual\">\n"
        f"          <geometry><box>"
        f"<size>{length:.3f} {width:.3f} {height:.3f}</size>"
        f"</box></geometry>\n"
        f"          <material><script>"
        f"<uri>file://media/materials/scripts/gazebo.material</uri>"
        f"<name>Gazebo/{color}</name></script></material>\n"
        f"        </visual>\n"
        f"      </link>\n"
        f"    </model>\n"
    )


def segment_box(name, x0, y0, x1, y1, width, height, z_center,
                collision, color, extend):
    """把 (x0,y0)->(x1,y1) 做成一个居中 box：长=段长+2*extend，沿段方向。"""
    dx, dy = x1 - x0, y1 - y0
    seg_len = math.hypot(dx, dy)
    if seg_len < 1e-6:
        return ""
    yaw = math.atan2(dy, dx)
    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    return box_model(name, cx, cy, yaw, seg_len + 2 * extend,
                     width, height, z_center, collision, color)


# ── 中心线：航向积分 ──────────────────────────────────────────────────

def offset_pt(px, py, th, off):
    """沿航向 th 的左法线 (-sin, cos) 偏移 off（正=左，负=右）。"""
    return px - math.sin(th) * off, py + math.cos(th) * off


def centerline_segments(segments, arc_ds):
    """
    展开中心线为「相邻点对」列表，每个元素 = ((x0,y0,t0), (x1,y1,t1))。
    直线段：整段作为若干共线小段；圆弧段：按弧长细分为小段。
    同时返回积分终点 (ex,ey,eth) 供闭合校验。
    """
    x, y, th = 0.0, 0.0, 0.0
    seg_pairs = []
    for seg in segments:
        if seg[0] == 'line':
            L = seg[1]
            n = max(1, int(round(L / arc_ds)))
            step = L / n
            for _ in range(n):
                x0, y0, t0 = x, y, th
                x += math.cos(th) * step
                y += math.sin(th) * step
                seg_pairs.append(((x0, y0, t0), (x, y, th)))
        elif seg[0] == 'arc':
            R, sweep_deg = seg[1], seg[2]
            sweep = math.radians(sweep_deg)
            n = max(1, int(round(abs(R * sweep) / arc_ds)))
            dth = sweep / n
            chord = R * abs(dth)
            for _ in range(n):
                x0, y0, t0 = x, y, th
                th_mid = th + dth / 2.0            # 中值航向积分，误差更小
                x += math.cos(th_mid) * chord
                y += math.sin(th_mid) * chord
                th += dth
                seg_pairs.append(((x0, y0, t0), (x, y, th)))
        else:
            raise ValueError(f"未知段类型: {seg[0]}")
    return seg_pairs, (x, y, th)


# ── 组装 world ────────────────────────────────────────────────────────

def _ccw(px, py, qx, qy, rx, ry):
    return (ry - py) * (qx - px) > (qy - py) * (rx - px)


def _seg_intersect(a, b, c, d):
    return (_ccw(a[0], a[1], c[0], c[1], d[0], d[1]) !=
            _ccw(b[0], b[1], c[0], c[1], d[0], d[1]) and
            _ccw(a[0], a[1], b[0], b[1], c[0], c[1]) !=
            _ccw(a[0], a[1], b[0], b[1], d[0], d[1]))


def validate(pairs, end, width):
    """三项底线校验：闭合残差 / 中心线自交 / 非相邻腿间距（防两走廊连通）。"""
    ex, ey, _ = end
    close_err = math.hypot(ex, ey)
    pts = [pairs[0][0][:2]] + [p[1][:2] for p in pairs]
    n = len(pts)

    # 自交：跳过共享端点的相邻段
    cross = 0
    for i in range(n - 1):
        for j in range(i + 2, n - 1):
            if i == 0 and j == n - 2:
                continue    # 闭合接缝处首尾相邻
            if _seg_intersect(pts[i], pts[i + 1], pts[j], pts[j + 1]):
                cross += 1

    # 非相邻腿最小间距：环上拓扑距离 >12m 的点对才参与（排除接缝假阳性）
    step = math.hypot(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]) or 1.0
    K = int(12.0 / step)
    leg_gap = 1e9
    for i in range(n):
        for j in range(i + 1, n):
            if min(j - i, n - (j - i)) <= K:
                continue
            leg_gap = min(leg_gap, math.hypot(pts[i][0] - pts[j][0],
                                              pts[i][1] - pts[j][1]))
    grass = leg_gap - width if leg_gap < 1e9 else float('inf')

    ok_close = close_err < 0.5
    ok_cross = cross == 0
    ok_gap = grass >= 0.5     # 两走廊间草地净距 <0.5m → 道路与自己连通，环失效
    print(f"[validate] 闭合残差 {close_err:.3f}m {'OK' if ok_close else 'FAIL'} | "
          f"中心线自交 {cross} 处 {'OK' if ok_cross else 'FAIL'} | "
          f"两腿草地净距 {grass:.2f}m {'OK' if ok_gap else 'FAIL(<0.5m)'}",
          file=sys.stderr)
    return ok_close and ok_cross and ok_gap


def build_world(preset_name, out_path):
    preset = PRESETS[preset_name]
    sections, width, arc_ds, spawn = (
        preset['sections'], preset['width'], preset['ds'], preset['spawn'])
    pairs, end = centerline_segments(sections, arc_ds)
    close_err = math.hypot(end[0], end[1])
    if not validate(pairs, end, width):
        print("[validate] 存在几何问题，仍写出文件供调试，请修正参数后重跑",
              file=sys.stderr)

    half = 0.5 * width
    curb_off = half + CURB_T / 2.0      # 路缘中心线到道路中心距离

    body = []
    for i, ((x0, y0, t0), (x1, y1, t1)) in enumerate(pairs):
        # 沥青：贴在中心线，宽 ROAD_WIDTH，visual-only
        body.append(segment_box(
            f"asphalt_{i}", x0, y0, x1, y1,
            width, 0.01, 0.005, False, "Black", JOINT_EXT))
        # 左路缘
        lx0, ly0 = offset_pt(x0, y0, t0, curb_off)
        lx1, ly1 = offset_pt(x1, y1, t1, curb_off)
        body.append(segment_box(
            f"curbL_{i}", lx0, ly0, lx1, ly1,
            CURB_T, CURB_H, CURB_H / 2.0, True, "Grey", JOINT_EXT))
        # 右路缘
        rx0, ry0 = offset_pt(x0, y0, t0, -curb_off)
        rx1, ry1 = offset_pt(x1, y1, t1, -curb_off)
        body.append(segment_box(
            f"curbR_{i}", rx0, ry0, rx1, ry1,
            CURB_T, CURB_H, CURB_H / 2.0, True, "Grey", JOINT_EXT))

    sx, sy, syaw = spawn
    tag = 'road' if preset_name == 'loop' else preset_name
    road_launch = ('loop_road.launch' if preset_name == 'loop'
                   else f'loop_{tag}_road.launch')
    goal_launch = ('loop_goal.launch' if preset_name == 'loop'
                   else f'loop_{tag}_goal.launch')
    desc = ('直道 + S形chicane(R3反弯) + 180°发夹弯(R2.5，内缘路缘半径0.85m)'
            if preset_name == 'extreme' else
            '直道 + 小曲率缓弯 + 适中 U 形弯')
    header = f"""<?xml version="1.0" ?>
<!--
  AUTO-GENERATED by scripts/gen_road_world.py —— 请勿手改本文件；
  改道路形状请编辑脚本顶部 PRESETS['{preset_name}'] 后重跑：
      python3 scripts/gen_road_world.py -p {preset_name}
  （注：XML 注释内不得出现连续两短横，故此处用单短横别名 -p）

  {preset_name}：首尾闭合的纯道路世界（无障碍），含 {desc}。
  路宽 {width:.1f}m，配 inflation_radius 0.7 → 净通行约 {width - 2 * 0.7:.1f}m。
  中心线数值积分闭合残差 {close_err:.3f}m（设计为点对称闭合）。
  出生点 ({sx:.1f}, {sy:.1f}) 朝 yaw={syaw:.2f}rad（沿第一段直道）。

  提示：本世界同时用于两套 launch ——
    · 无定位沿路：{road_launch}（follow_road=true，跟路缘走廊）
    · 有定位终点：{goal_launch} （follow_road=false，RViz 点 2D Nav Goal）
-->
<sdf version="1.6">
  <world name="loop_{tag}">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <include><uri>model://sun</uri></include>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
          <geometry><plane><normal>0 0 1</normal>
            <size>{GROUND_SIZE:.0f} {GROUND_SIZE:.0f}</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal>
            <size>{GROUND_SIZE:.0f} {GROUND_SIZE:.0f}</size></plane></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Green</name></script></material>
        </visual>
      </link>
    </model>

"""
    footer = "  </world>\n</sdf>\n"
    with open(out_path, "w") as f:
        f.write(header + "".join(body) + footer)
    print(f"wrote {out_path}  ({len(pairs)} 段 × 3 = {len(pairs) * 3} 个 box)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="参数化生成闭合环形道路 world")
    ap.add_argument("-p", "--preset", choices=sorted(PRESETS), default=None,
                    help="场景预设（默认 None = 生成全部）")
    ap.add_argument("-o", "--out", default=None,
                    help="输出 .world 路径（仅单 preset 时可用；默认 ../src/unk_nav_sim/worlds/<preset>.world）")
    args = ap.parse_args()
    presets = [args.preset] if args.preset else sorted(PRESETS)
    if args.out and len(presets) > 1:
        print("-o 仅单 preset 时可用", file=sys.stderr); sys.exit(2)
    for name in presets:
        tag = 'road' if name == 'loop' else name
        out = args.out or os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "src", "unk_nav_sim", "worlds", f"loop_{tag}.world")
        build_world(name, out)
