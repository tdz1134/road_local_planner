#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_road_world.py —— 参数化生成「首尾闭合道路」Gazebo 世界（SDF）+ 配套 launch。

背景：SDF 原生只有 box/cylinder/sphere/plane/mesh，没有「圆弧/曲线」碰撞体。
所以道路（含弯道）必须用「密集小直线 box 段」逼近曲率。本脚本用航向积分
（heading integration）参数化定义道路中心线，再两侧各按走廊剖面偏移生成路缘
墙段，输出一份完整 .world。改形状 = 改下面 PRESETS 里对应 preset 的
width / bias / sections / ds 参数，重跑即可（-p <preset>）。

────────────────────────────────────────────────────────────────────────
道路 = 两侧路缘墙夹出的自由走廊（与 road_world.world 的「路线 A」一致）：
  · 激光打到路缘 → grid_node 生成 free 带 = 道路；沿路模式据此跟路。
  · 路缘全为凸形小 box 段，接缝靠 JOINT_EXT 延伸压合，防激光漏过缝隙。
  · 沥青条 visual-only、贴地 z≈0.005 < grid_node z_min=0.1，激光打不到 → 不污染栅格。
  · 本脚本只出「纯道路」；走廊内带障碍的场景在 gen_obstacle_world.py。
────────────────────────────────────────────────────────────────────────

三个自由度，对应三类真实路况：
  · sections  中心线平面形状（直道 / 圆弧 / 180° 半圆）。
  · width     总宽，可以是标量，也可以是 [(s, 宽), ...] 关键帧（分段线性）
              → 变路宽（收口、拓宽）。
  · bias      中心线相对走廊几何中线的偏移（左正），同样可给关键帧
              → 两侧特性不一样。它的真正用处：弯道内侧缘半径 = R - 内侧距，
                把中心线压向内侧就能造出「一侧近直角、一侧大圆弧」的非对称弯，
                这正是真实路口两侧的样子（对称路宽做不到）。

坐标约定：pose = x y z roll pitch yaw；box 的 <size>=长(x) 宽(y) 高(z)。
s = 沿中心线弧长 m，自积分起点 (0,0, 航向0) 起算。

用法：
    python3 gen_road_world.py                 # 生成全部预设（world + launch）
    python3 gen_road_world.py -p taper        # 只生成 taper
    python3 gen_road_world.py -p taper --ascii  # 只看俯视示意，不写文件
    python3 gen_road_world.py --no-launch     # 只出 world
（world 文件名规则：loop_<tag>.world；preset=loop 时 tag=road 保持历史兼容。
  launch 只在目标文件不存在、或本身就是本脚本生成时才写 —— 手改过的 launch 里
  往往有比脚本更详细的实测告警，不能被静默覆盖，跳过时会打印提示。）
"""

import argparse
import math
import os
import sys
import xml.parsers.expat

# ══════════════════════ 可调参数（改这里即可）══════════════════════════
CURB_T = 0.3            # 路缘墙厚 m
CURB_H = 1.5            # 路缘墙高 m（须 > grid_node z_min，激光才打得到；沿用现有 1.5）
JOINT_EXT = 0.18        # 每段两端各延伸 m，压住相邻段接缝（防激光从缝里漏过去）
GROUND_SIZE = 120.0     # 地面尺寸 m（要罩住整条环路）
# ══════════════════ 与 nav_params*.yaml / grid_params.yaml 保持同步 ══════════
# 这两个值改动时，务必同步 src/unk_nav/config/nav_params{,_road}.yaml，
# 否则脚本判「能不能过」用的是一套参数、车跑起来用的是另一套。
INFLATION = 0.7         # inflation_radius：膨胀半径 m，两侧共吃 2×0.7 = 1.4m
ROBOT_R = 0.46          # robot_radius：车体半径 m
# ════════════════════════ 几何下限（防自伤，非性能指标）════════════════════
MIN_CURB_R = 0.35       # 内侧路缘最小曲率半径 m：小于此内缘折返，栅格里糊成一坨
MIN_NET_W = 0.2         # 最小净通行宽 m：width - 2×INFLATION 低于此 = 走廊自己封死
MAX_LAT_STEP = CURB_T / 2.0   # 变宽处相邻 box 的最大横向台阶 m（须 < 墙厚）
# ═══════════════════════════════════════════════════════════════════════════

# 预设场景：width/bias/ds/spawn(_s)/sections。
# 段序列元素：('line', 长度) 或 ('arc', 半径, 圆心角deg)。正圆心角=左转，负=右转。
# 闭合条件：前半序列净转角 = 180°，后半重复同一序列 → 点对称自动闭合。
# width/bias 给列表时是「沿 s 的分段线性关键帧」，超出首末取端点值。
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
    # 基线回归场：标准「体育场」环 —— 两条 24m 直道 + 两个 180° 半圆（R9）。
    # 与 loop（两个 135° U 形弯 + 45° 缓弯）的区别是这里曲率沿整段半圆恒定、
    # 且掉头是连续 180° 而非两段拼接。改跟路/控制参数后先跑它：几何最简单，
    # 这里都不稳定就不是场景难，是实现退化了。路宽 5m → 净通行 3.6m。
    # 内缘半径 = 9 - 2.5 = 6.5m，外缘 = 11.5m，都远离折返下限 MIN_CURB_R。
    'stadium': {
        'width': 5.0, 'ds': 0.35, 'spawn_s': 2.0,
        'sections': [
            ('line', 24.0),
            ('arc', 9.0, 180.0),    # 半圆掉头
            ('line', 24.0),
            ('arc', 9.0, 180.0),
        ],
    },
    # 变路宽闭合环：中心线是「直道22 + 缓弯R12(45°) + 直道10 + U形弯R5(135°)」重复
    # 两遍（点对称），总长 106.4m。总宽沿 s 渐变：
    #   7.0 →(收口)→ 3.2 →(拓宽)→ 7.0 →(收口)→ 3.2 →(拓宽)→ 7.0
    # 测的是「走廊宽度本身是变量」时跟路还成不成立：收窄处两侧路缘同时向中线靠拢，
    # 前瞻 7m 内看到的可通行断面一直在变；拓宽处相反（更容易外侧切角）。
    # 两条硬约定（都由 validate 兜着，写错会直接报 FAIL）：
    #   · 收口/拓宽只落在直道上（s 4~14、31.4~40 及其 +53.2 的镜像），斜率 0.44
    #     → 半宽横向台阶 0.44×ds/2 = 0.088m < MAX_LAT_STEP 0.15，激光穿不过接缝；
    #   · 剖面周期 = 半环长 53.2m（w(s) == w(s+53.2)），否则闭合缝上会出现 0.5m 级
    #     的横向跳变 —— 那既不像任何真实路，也会在栅格上留个洞。
    # 最窄 3.2m 是下限：净通行 = 3.2 - 1.4 = 1.8m（车侧 0.44m），再窄就自己封死。
    # 窄段 3.2m 跨过了缓弯 R12（内缘 10.4m）没问题；宽段 7.0m 跨 U 形弯 R5 时
    # 内缘 = 5 - 3.5 = 1.5m > MIN_CURB_R，不折返但已经很挤 —— 这处本身就是考点。
    'taper': {
        'ds': 0.40, 'spawn_s': 2.0, 'sections': [
            ('line', 22.0), ('arc', 12.0, 45.0),
            ('line', 10.0), ('arc', 5.0, 135.0),
        ] * 2,
        'width': [(0.0, 7.0), (4.0, 7.0), (14.0, 3.2), (31.4, 3.2),
                  (40.0, 7.0), (57.2, 7.0), (67.2, 3.2), (84.6, 3.2),
                  (93.2, 7.0), (106.4, 7.0)],
        'bias': 0.0,
    },
    # 两侧特性不一样的闭合环：同一环上并存「大圆弧弯」和「近直角弯」，且直角弯
    # 处把中心线压向外侧（bias=-1.5 → 左缘距 1.5、右缘距 4.5）：
    #   · 近直角弯（中心线 R2.7，前半 61.1m 处与后半 106.8m 处各一次）：
    #     左缘（内侧）半径 2.7-1.5 = 1.2m ≈ 直角，右缘（外侧）半径 2.7+4.5 = 7.2m
    #     大圆弧。同一次转向里两侧曲率差 6 倍。
    #   · 大圆弧弯（R12）：两侧半径 10.5 / 13.5，几乎平行。
    # 为什么值得单开一个场景：跟路子目标只从激光走廊几何推，不认「中心线」这个概念。
    # 两侧曲率差 6 倍时，走廊中线明显不在车正前方，对称环里看不到的横向偏差会暴露
    # 出来；而按几何中线跑（很多实现默认如此）在直角处必然吃内侧路缘。
    # bias 的斜坡必须在直道上完成（validate 的折返检查会把伸进弯内的斜坡揪出来：
    # 第一版就是它报的「s=44.9 处内缘半径 0.25m」），且左右对称地出现在两处直角弯。
    # 内缘 1.2m 仍 > MIN_CURB_R，路缘不折返；总宽 6m 不变，净通行 4.6m。
    'asymturn': {
        'ds': 0.25, 'spawn_s': 3.0, 'width': 6.0,
        'sections': [
            ('line', 22.0),
            ('arc', 12.0, 45.0),    # 大圆弧弯：两侧近乎平行
            ('line', 10.0),
            ('arc', 2.7, 90.0),     # 近直角弯：内缘 R1.2 / 外缘 R7.2
            ('line', 6.0),
            ('arc', 12.0, 45.0),    # 补足前半净转角 180°
        ] * 2,
        # 半环 61.1m：直角弯占 41.4~45.7，偏置斜坡 38→41 与 46→49 都在直道上
        # （斜率 0.5 → 横向台阶 0.125m，刚好不上 MAX_LAT_STEP 的 FAIL），
        # 后半 106.8~111.1 是同一形状平移 61.1。
        'bias': [(0.0, 0.0), (38.0, 0.0), (41.0, -1.5), (46.0, -1.5),
                 (49.0, 0.0), (99.0, 0.0), (102.0, -1.5), (107.0, -1.5),
                 (110.0, 0.0), (130.0, 0.0)],
    },
}

WORLD_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "..", "src", "unk_nav_sim", "worlds")
LAUNCH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "..", "src", "unk_nav_sim", "launch")
GEN_MARK = "AUTO-GENERATED by"   # 见过这个标记的 launch 才允许被覆盖
FORCE_LAUNCH = False            # --force-launch：连手改过的 launch 也重写（默认关）


# ── 沿 s 的剖面：分段线性 ─────────────────────────────────────────────

class Profile:
    """标量或 [(s, v), ...] 关键帧 → 沿弧长的分段线性函数（超出首末取端点值）。"""

    def __init__(self, spec, name="profile"):
        if isinstance(spec, (int, float)):
            self.keys, self.const = None, float(spec)
        elif spec:
            ks = sorted(spec, key=lambda kv: kv[0])
            for (a, _), (b, _v) in zip(ks, ks[1:]):
                if math.isclose(a, b):
                    raise ValueError(f"{name}: 关键帧 s={a} 重复，无法插值")
            self.keys, self.const = ks, None
        else:
            raise ValueError(f"{name}: 空剖面")

    def __call__(self, s):
        if self.const is not None:
            return self.const
        ks = self.keys
        if s <= ks[0][0]:
            return ks[0][1]
        if s >= ks[-1][0]:
            return ks[-1][1]
        lo, hi = 0, len(ks) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if ks[mid][0] <= s:
                lo = mid
            else:
                hi = mid
        s0, v0 = ks[lo]
        s1, v1 = ks[hi]
        return v0 + (v1 - v0) * (s - s0) / (s1 - s0)

    def span(self, s0, s1, n=24):
        """[s0,s1] 上的 (最小值, 最大值)，用于逐段校验（关键帧可能落在区间内部）。"""
        vals = [self(s0), self(s1)]
        if self.keys:
            vals += [v for k, v in self.keys if s0 <= k <= s1]
        vals += [self(s0 + (s1 - s0) * i / n) for i in range(1, n)]
        return min(vals), max(vals)

    @property
    def const_value(self):
        return self.const


class Corridor:
    """
    走廊剖面：总宽 width + 中心线偏置 bias（左正）→ 左缘距 left(s)、右缘距 right(s)。

    bias 的意义不是「歪着画」，而是把中心线放到走廊的某一侧去：弯道内侧缘半径
    = R - 内侧距，压近内侧就能把同一个 R 的弯做成「一侧近直角、一侧大圆弧」。
    注意 bias 不影响总宽，也就不影响「净通行够不够」这件事 —— 两者是独立参数。
    """

    def __init__(self, width, bias=None):
        self.w = Profile(width, "width")
        self.b = Profile(0.0 if bias is None else bias, "bias")

    def width(self, s):
        return self.w(s)

    def left(self, s):
        return 0.5 * self.w(s) + self.b(s)

    def right(self, s):
        return 0.5 * self.w(s) - self.b(s)

    def is_const(self):
        return self.w.const_value is not None and self.b.const_value is not None

    @staticmethod
    def of(spec):
        """兼容旧调用：直接传路宽标量也能用。"""
        return spec if isinstance(spec, Corridor) else Corridor(spec)


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


def cylinder_model(name, cx, cy, r, h, z_center, color):
    return (
        f"    <model name=\"{name}\">\n"
        f"      <static>true</static>\n"
        f"      <pose>{cx:.3f} {cy:.3f} {z_center:.3f} 0 0 0</pose>\n"
        f"      <link name=\"link\">\n"
        f"        <collision name=\"collision\">\n"
        f"          <geometry><cylinder>"
        f"<radius>{r:.3f}</radius><length>{h:.3f}</length>"
        f"</cylinder></geometry>\n"
        f"        </collision>\n"
        f"        <visual name=\"visual\">\n"
        f"          <geometry><cylinder>"
        f"<radius>{r:.3f}</radius><length>{h:.3f}</length>"
        f"</cylinder></geometry>\n"
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


def ground_plane():
    return f"""    <model name="ground_plane">
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


def polyline(pairs):
    """pairs → ([(s, x, y, th), ...], 总长)。相邻对共享端点，故逐段累加。"""
    nodes = [(0.0, pairs[0][0][0], pairs[0][0][1], pairs[0][0][2])]
    s = 0.0
    for (a, b) in pairs:
        s += math.hypot(b[0] - a[0], b[1] - a[1])
        nodes.append((s, b[0], b[1], b[2]))
    return nodes, s


def sample(nodes, s):
    """按弧长取中心线位姿（二分 + 线性插值）。弯道处 lat 偏移因此连续。"""
    if s <= nodes[0][0]:
        return nodes[0][1:]
    if s >= nodes[-1][0]:
        return nodes[-1][1:]
    lo, hi = 0, len(nodes) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if nodes[mid][0] <= s:
            lo = mid
        else:
            hi = mid
    s0, x0, y0, t0 = nodes[lo]
    s1, x1, y1, t1 = nodes[hi]
    k = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)
    return x0 + (x1 - x0) * k, y0 + (y1 - y0) * k, t0 + (t1 - t0) * k


def section_spans(sections):
    """各段的 [s0, s1] 与可读描述，供 world 头部注释和报告用（不重新积分）。"""
    out, s = [], 0.0
    for seg in sections:
        if seg[0] == 'line':
            L, desc = seg[1], f"直道 {seg[1]:.1f}m"
        else:
            R, sweep = seg[1], math.radians(seg[2])
            L, desc = abs(R * sweep), f"弧 R{R:.1f} {seg[2]:+.0f}°"
        out.append((s, s + L, desc))
        s += L
    return out, s


def road_body(pairs, nodes, prof, prefix=""):
    """
    沥青（visual-only）+ 左右路缘墙。宽度沿 s 变化时，每个小 box 取该小段中点
    的剖面值（段内视为等宽）：横向台阶 = ds × |dw/ds| / 2，由 validate 卡住上限。
    """
    body = []
    for i, ((x0, y0, t0), (x1, y1, t1)) in enumerate(pairs):
        # 每小段的 s 取中点：用 nodes 累加值，避免自己再算一遍弧长
        sm = 0.5 * (nodes[i][0] + nodes[i + 1][0])
        wl, wr = prof.left(sm), prof.right(sm)
        # 沥青条要真的铺在走廊上：有 bias 时走廊中线不在中心线上
        ax0, ay0 = offset_pt(x0, y0, t0, 0.5 * (wl - wr))
        ax1, ay1 = offset_pt(x1, y1, t1, 0.5 * (wl - wr))
        body.append(segment_box(
            f"{prefix}asphalt_{i}", ax0, ay0, ax1, ay1,
            wl + wr, 0.01, 0.005, False, "Black", JOINT_EXT))
        for edge, tag in ((wl, "curbL"), (-wr, "curbR")):
            # 路缘中心线 = 走廊边线再往外挪半个墙厚，免得墙内表面侵进路面
            ax, ay = offset_pt(x0, y0, t0, _curb_off(edge))
            bx, by = offset_pt(x1, y1, t1, _curb_off(edge))
            body.append(segment_box(
                f"{prefix}{tag}_{i}", ax, ay, bx, by,
                CURB_T, CURB_H, CURB_H / 2.0, True, "Grey", JOINT_EXT))
    return body


def _curb_off(edge):
    """走廊边线 → 路缘墙中心线：往外侧挪半个墙厚（edge 带符号，正=左侧）。"""
    return edge + (CURB_T / 2.0 if edge >= 0 else -CURB_T / 2.0)


# ── 几何校验 ──────────────────────────────────────────────────────────

def _orient(px, py, qx, qy, rx, ry):
    """cross(q-p, r-p) 的符号，按坐标量级归一后与 EPS 比：共线时给 0而不是随机符号。"""
    cross = (qx - px) * (ry - py) - (qy - py) * (rx - px)
    scale = abs(qx - px) * abs(ry - py) + abs(qy - py) * abs(rx - px)
    rel = cross / scale if scale > 0 else 0.0
    if abs(rel) < 1e-12:
        return 0
    return 1 if rel > 0 else -1


def _seg_intersect(a, b, c, d):
    """
    严格穿越判定（共享端点与相切都不算）。
    中心线被离散成共 ds 的小段，同一腿上的相邻子段完全共线：不带 EPS 的方向
    测试会在 1e-17 量级上随机翻转，把一条直道自己判成“自交 5 处”。
    共线端点（T 形相接）也返回 False：那是退化身形，由腿间距检查接手。
    """
    x1 = _orient(*a, *b, *c)
    x2 = _orient(*a, *b, *d)
    y1 = _orient(*c, *d, *a)
    y2 = _orient(*c, *d, *b)
    if x1 == x2 == y1 == y2 == 0:
        return False        # 四端点共线：同一腿的离散子段，不是交叉
    return (x1 * x2 < 0) and (y1 * y2 < 0)


def validate(pairs, end, prof):
    """
    六项底线校验，分两类：
      拓扑类（错了场景就不成立，必须修参数）：闭合残差 / 中心线自交 / 两腿草地净距
      自伤类（错了生成出来的东西在栅格里不是你以为的形状）：内侧路缘折返 /
      最小净通行宽 / 变宽处的横向台阶
    prof 可传 Corridor 或路宽标量（旧调用写法）。
    """
    prof = Corridor.of(prof)
    nodes, _total = polyline(pairs)
    ex, ey, _ = end
    close_err = math.hypot(ex, ey)
    pts = [pairs[0][0][:2]] + [p[1][:2] for p in pairs]
    n = len(pts)
    step = math.hypot(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]) or 1.0

    # 1) 中心线自交：跳过共享端点的相邻段
    cross = 0
    for i in range(n - 1):
        for j in range(i + 2, n - 1):
            if i == 0 and j == n - 2:
                continue    # 闭合接缝处首尾相邻
            if _seg_intersect(pts[i], pts[i + 1], pts[j], pts[j + 1]):
                cross += 1

    # 2) 非相邻腿最小草地净距：环上拓扑距离 >12m 的点对才参与（排除接缝假阳性）
    K = int(12.0 / step)
    leg_gap = 1e9
    for i in range(n):
        for j in range(i + 1, n):
            if min(j - i, n - (j - i)) <= K:
                continue
            # 两腿各自要减去自己的半宽才算「草地净距」，取该 s 处的实际剖面
            gi = max(prof.left(nodes[i][0]), prof.right(nodes[i][0]))
            gj = max(prof.left(nodes[j][0]), prof.right(nodes[j][0]))
            leg_gap = min(leg_gap, math.hypot(pts[i][0] - pts[j][0],
                                              pts[i][1] - pts[j][1]) - gi - gj)
    grass = leg_gap if leg_gap < 1e9 else float('inf')

    # 3) 内侧路缘折返 4) 最小净宽 5) 变宽处横向台阶 / 偏置过头：逐节点判
    fold_at, min_net, step_at = None, 1e9, None
    for i in range(n):
        s = nodes[i][0]
        min_net = min(min_net, prof.width(s) - 2 * INFLATION)
        dth = nodes[i + 1][3] - nodes[i][3] if i + 1 < n else \
            nodes[i][3] - nodes[i - 1][3]
        if abs(dth) > 1e-9:                     # 该处是弯的：内缘半径 = R - 内侧距
            r_loc = step / abs(dth)             # 弦长/圆心角 ≈ R（略小，偏保守）
            inner = prof.left(s) if dth > 0 else prof.right(s)
            if r_loc - inner < MIN_CURB_R and fold_at is None:
                fold_at = (s, r_loc - inner)
        if i:
            # 相邻 box 的侧向台阶：变宽太快会留缝让激光漏过去；偏置过头会让某一侧
            # 缘距小于半墙厚，墙会翻到中心线另一侧去
            d_lat = max(abs(prof.left(nodes[i][0]) - prof.left(nodes[i - 1][0])),
                        abs(prof.right(nodes[i][0]) - prof.right(nodes[i - 1][0])))
            tight = min(prof.left(s), prof.right(s))
            if (d_lat > MAX_LAT_STEP or tight < CURB_T / 2.0) and step_at is None:
                step_at = (nodes[i - 1][0], d_lat, tight)
    wl0, wr0 = prof.left(0.0), prof.right(0.0)

    ok_close = close_err < 0.5
    ok_cross = cross == 0
    ok_gap = grass >= 0.5
    ok_fold = fold_at is None
    ok_net = min_net >= MIN_NET_W
    ok_step = step_at is None
    print(f"[validate] 闭合残差 {close_err:.3f}m {'OK' if ok_close else 'FAIL'} | "
          f"中心线自交 {cross} 处 {'OK' if ok_cross else 'FAIL'} | "
          f"两腿草地净距 {grass:.2f}m {'OK' if ok_gap else 'FAIL(<0.5m)'}",
          file=sys.stderr)
    print(f"[validate] 最小净通行 {min_net:.2f}m（车侧 {(min_net - 2 * ROBOT_R) / 2:+.2f}m）"
          f" {'OK' if ok_net else f'FAIL(<{MIN_NET_W}m，inflation 已把走廊封死)'} | "
          f"内侧路缘折返 {'OK' if ok_fold else f'FAIL s={fold_at[0]:.1f} 处内缘半径 {fold_at[1]:.2f}m < {MIN_CURB_R}m'} | "
          f"横向台阶/边距 {'OK' if ok_step else f'FAIL s={step_at[0]:.1f} 台阶{step_at[1]:.2f}m 最小边距{step_at[2]:.2f}m'} | "
          f"s=0 左缘距 {wl0:.2f} 右缘距 {wr0:.2f}",
          file=sys.stderr)
    return all([ok_close, ok_cross, ok_gap, ok_fold, ok_net, ok_step])


def check_xml(path):
    """
    写后自检 well-formed。这里只解析本脚本刚生成的文件（不是外部输入），但仍显式
    拒绝实体声明、让外部实体解析失败：既避开 DTD/实体那类解析风险，也顺带卡住
    「XML 注释内出现连续两短横」这类硬错误 —— 上次就是靠它发现的。
    """
    parser = xml.parsers.expat.ParserCreate()

    def reject_entity(*_args):
        raise ValueError("含实体声明，拒绝")

    parser.EntityDeclHandler = reject_entity
    parser.ExternalEntityRefHandler = lambda *_a: False
    with open(path, "rb") as f:
        parser.ParseFile(f)


def edge_curvature(nodes, prof):
    """
    左右两侧路缘各自「最弯处」的半径 (rL, rR) —— 这是「两侧特性不一样」唯一可信的
    量化指标：两侧半径差几倍，车看到的可通行边界就差几倍弯。
    局部半径用弦长/航向增量估（略小于真值，偏保守），只在弯曲处统计。
    """
    rL = rR = float('inf')
    for i in range(len(nodes) - 1):
        s = nodes[i][0]
        dth = nodes[i + 1][3] - nodes[i][3]
        if abs(dth) < 1e-9:
            continue
        step = math.hypot(nodes[i + 1][1] - nodes[i][1],
                          nodes[i + 1][2] - nodes[i][2])
        r = step / abs(dth)
        if dth > 0:                 # 左转：左缘在内（半径小），右缘在外
            rL, rR = min(rL, r - prof.left(s)), min(rR, r + prof.right(s))
        else:                       # 右转：反过来
            rL, rR = min(rL, r + prof.left(s)), min(rR, r - prof.right(s))
    return rL, rR


def describe(prof, sections, nodes, total):
    """world 头部与终端报告共用的几何摘要：段表 + 宽度/偏置范围 + 两侧弯曲度。"""
    spans, _ = section_spans(sections)
    wmin, wmax = prof.w.span(0.0, total)
    bmin, bmax = prof.b.span(0.0, total)
    rL, rR = edge_curvature(nodes, prof)
    inf = float('inf')
    lines = [f"环长 {total:.1f}m，中心线段序："]
    for s0, s1, desc in spans:
        lines.append(f"    s {s0:6.1f} → {s1:6.1f}  {desc}")
    if wmin == wmax:
        lines.append(f"路宽恒定 {wmax:.1f}m（无障碍净通行 {wmax - 2 * INFLATION:.1f}m"
                     f" = 宽 - 2×inflation {INFLATION}）")
    else:
        lines.append(f"路宽沿 s 变化 {wmin:.1f} ~ {wmax:.1f}m，"
                     f"最小净通行 {wmin - 2 * INFLATION:.1f}m")
    if (bmin, bmax) != (0.0, 0.0):
        lines.append(f"中心线偏置（左正）{bmin:+.1f} ~ {bmax:+.1f}m → 左缘距 "
                     f"{wmin / 2 + bmin:.1f} ~ {wmax / 2 + bmax:.1f}m、右缘距 "
                     f"{wmin / 2 - bmax:.1f} ~ {wmax / 2 - bmin:.1f}m")
    if rL == inf and rR == inf:
        lines.append("两侧路缘全程无弯曲（纯直道环）")
    else:
        f = lambda v: "直" if v == inf else f"{v:.1f}m"
        ratio = "" if (rL == inf or rR == inf) else \
            f"（差 {max(rL, rR) / min(rL, rR):.1f} 倍）" if min(rL, rR) > 0 else ""
        lines.append(f"两侧路缘最弯处半径：左 {f(rL)}、右 {f(rR)}{ratio}")
    return lines


# ── 俯视示意图 ────────────────────────────────────────────────────────

def render_ascii(nodes, total, prof, obs=(), spawn=None, dx=0.5, dy=1.0,
                 step=0.25):
    """
    终端字符约 2:1，故 dx = dy/2。step 必须明显小于 dx，否则斜段会漏格。
    obs 为 gen_obstacle_world.expand() 的原子体列表；纯道路传空即可。
    返回 (grid, nh, nw)，拼字符串的活留给调用方。
    """
    pts = []
    for i in range(int(total / step) + 1):
        s = i * step
        x, y, th = sample(nodes, s)
        pts.append((s, x, y, th))
    boxes, circles = [], []
    for ob in obs:
        x, y, th = sample(nodes, ob['s'])
        cx, cy = offset_pt(x, y, th, ob['lat'])
        if ob['shape'] == 'cyl':
            circles.append((cx, cy, ob['r']))
        else:
            corners = []
            for lx, ly in ((ob['half_l'], ob['half_w']), (ob['half_l'], -ob['half_w']),
                           (-ob['half_l'], ob['half_w']), (-ob['half_l'], -ob['half_w'])):
                a = th + ob['yaw']
                corners.append((cx + lx * math.cos(a) - ly * math.sin(a),
                                cy + lx * math.sin(a) + ly * math.cos(a)))
            boxes.append(corners)
    allx = [p[1] for p in pts] + [c[0] for b in boxes for c in b] + \
           [c[0] - c[2] for c in circles] + [c[0] + c[2] for c in circles]
    ally = [p[2] for p in pts] + [c[1] for b in boxes for c in b] + \
           [c[1] - c[2] for c in circles] + [c[1] + c[2] for c in circles]
    x0, x1 = min(allx) - 1.5, max(allx) + 1.5
    y0, y1 = min(ally) - 1.5, max(ally) + 1.5
    nw = int((x1 - x0) / dx) + 1
    nh = int((y1 - y0) / dy) + 1
    grid = [[' '] * nw for _ in range(nh)]

    def put(px, py, ch):
        c, r = int((px - x0) / dx), int((y0 + (y1 - y0) - py) / dy)
        if 0 <= r < nh and 0 <= c < nw:
            # 越往“重要”的字符越不被覆盖：O > # > S > . > 空格
            # （障碍排在路缘之上：否则紧贴路缘的封锁墙会被路缘字占位遮掉）
            rank = {' ': 0, '.': 1, 'S': 2, '#': 3, 'O': 4}
            if rank[grid[r][c]] <= rank[ch]:
                grid[r][c] = ch

    for (s, x, y, th) in pts:
        wl, wr = prof.left(s), prof.right(s)
        put(*offset_pt(x, y, th, wl + CURB_T / 2.0), '#')
        put(*offset_pt(x, y, th, -(wr + CURB_T / 2.0)), '#')
        nfill = max(1, int((wl + wr) / 0.25))
        for j in range(nfill + 1):
            lx, ly = offset_pt(x, y, th, -wr + (wl + wr) * j / nfill)
            put(lx, ly, '.')
    for (cx, cy, r) in circles:
        for a in [i * 0.4 for i in range(16)]:
            put(cx + r * math.cos(a), cy + r * math.sin(a), 'O')
        put(cx, cy, 'O')
    for corners in boxes:
        # 填充内部而不是只画轮廓：block 这种「沿路 0.6m × 横向 4.1m」的薄墙，
        # 只画轮廓会退化成两条竖线，看图的人分不清它是一堵墙还是两根柱子
        c0, c1, c2 = corners[0], corners[1], corners[2]
        nu = max(1, int(math.hypot(c1[0] - c0[0], c1[1] - c0[1]) / 0.2))
        nv = max(1, int(math.hypot(c2[0] - c0[0], c2[1] - c0[1]) / 0.2))
        for iu in range(nu + 1):
            for iv in range(nv + 1):
                u, v = iu / nu, iv / nv
                px = c0[0] + u * (c1[0] - c0[0]) + v * (c2[0] - c0[0])
                py = c0[1] + u * (c1[1] - c0[1]) + v * (c2[1] - c0[1])
                put(px, py, 'O')
    if spawn:                     # 出生点最后落笔，否则会被走廊底色盖掉
        put(spawn[0], spawn[1], 'S')
    return grid, nh, nw


def print_ascii(nodes, total, prof, obs=(), spawn=None, title=""):
    grid, nh, nw = render_ascii(nodes, total, prof, obs, spawn)
    print(f"\n俯视示意 [{title}]  {nw}×{nh} 格（横 0.5m/格、纵 1.0m/格）"
          f"      . 走廊   # 路缘   O 障碍   S 出生点")
    print("\n".join(''.join(r).rstrip() for r in grid))


# ── launch 输出 ───────────────────────────────────────────────────────

LAUNCH_ROAD_BLURB = (
    "沿路模式（follow_road=true）：NavCore 从路缘走廊几何推车体系前瞻子目标，\n"
    "        不读定位与全局终点；障碍由下游 A* 负责绕行，绕不过就如实 RECOVERY/ABORT。")
LAUNCH_GOAL_BLURB = (
    "终点模式（follow_road=false）：RViz 点 2D Nav Goal，障碍绕行是纯 A* 行为。\n"
    "        注意终点要挑在走廊内且距障碍不小于 goal_clear_radius，否则斜穿草地。")


def writable_by_gen(path):
    """目标文件不存在、或本身就是脚本生成过（含 GEN_MARK）的才允许写。"""
    if FORCE_LAUNCH:
        return True
    if not os.path.exists(path):
        return True
    with open(path) as f:
        return GEN_MARK in f.read()


def build_launch(name, spawn, world_file, out_path, mode, notes, extra_args=""):
    """
    mode='road' 沿路（无定位）；mode='goal' 终点导航（有定位，RViz 点目标）。
    notes 是往头部注释里追加的若干行（各场景自己的实测告警），可为空。
    返回 False 表示为了保住手改内容而跳过。
    """
    sx, sy, syaw = spawn
    world_base = os.path.basename(world_file)
    if mode == 'road':
        cfg = "$(find unk_nav)/config/nav_params_road.yaml"
        loc = ('  <node name="localization_node" pkg="unk_nav_sim" '
               'type="localization_node"\n'
               '        output="screen" if="$(arg use_localization)">\n'
               '    <param name="model_name" value="scout/"/>\n'
               '  </node>\n')
        extra = '  <arg name="use_localization" default="false"/>\n' + extra_args
        fixed = "base_link"
        blurb = LAUNCH_ROAD_BLURB
    else:
        cfg = "$(find unk_nav)/config/nav_params.yaml"
        loc = ('  <node name="localization_node" pkg="unk_nav_sim" '
               'type="localization_node"\n'
               '        output="screen">\n'
               '    <param name="model_name" value="scout/"/>\n'
               '  </node>\n')
        extra = '' + extra_args
        fixed = "odom"
        blurb = LAUNCH_GOAL_BLURB
    note_txt = "".join(f"  {t}\n" for t in notes)
    content = f"""<?xml version="1.0"?>
<!--
  {GEN_MARK} {os.path.basename(sys.argv[0])} —— 请勿手改本文件，改场景请回脚本。
  {os.path.basename(out_path)}：{blurb}
  一键启动：Gazebo({world_base}) + Scout v2 + 栅格 + 导航 + RViz

{note_txt}
  RViz Fixed Frame 需为 {fixed}。

  用法：
    roslaunch unk_nav_sim {os.path.basename(out_path)}
    roslaunch unk_nav_sim {os.path.basename(out_path)} gui:=false    # 无头模式
-->
<launch>

  <arg name="gui" default="true"/>
{extra}
  <!-- 出生点：第一段直道上，与 world 内中心线取样一致（不要手改） -->
  <arg name="x"   default="{sx:.3f}"/>
  <arg name="y"   default="{sy:.3f}"/>
  <arg name="yaw" default="{syaw:.5f}"/>

  <!-- ══════════════════════════════════════════════════════════ -->
  <!-- 1. Gazebo 世界                                              -->
  <!-- ══════════════════════════════════════════════════════════ -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name"   value="$(find unk_nav_sim)/worlds/{world_base}"/>
    <arg name="paused"       value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui"          value="$(arg gui)"/>
    <arg name="headless"     value="false"/>
    <arg name="debug"        value="false"/>
  </include>

  <!-- ══════════════════════════════════════════════════════════ -->
  <!-- 2. Scout v2（含 Velodyne HDL-32E + robot_state_publisher）  -->
  <!-- ══════════════════════════════════════════════════════════ -->
  <include file="$(find scout_gazebo_sim)/launch/spawn_scout_v2.launch">
    <arg name="x"   value="$(arg x)"/>
    <arg name="y"   value="$(arg y)"/>
    <arg name="z"   value="0.0"/>
    <arg name="yaw" value="$(arg yaw)"/>
  </include>

  <!-- ══════════════════════════════════════════════════════════ -->
  <!-- 3. 定位（Gazebo 真值 → /odom + TF）                         -->
  <!-- ══════════════════════════════════════════════════════════ -->
{loc}
  <!-- ══════════════════════════════════════════════════════════ -->
  <!-- 4. 栅格节点（/velodyne_points → /local_grid）               -->
  <!-- ══════════════════════════════════════════════════════════ -->
  <node name="grid_node" pkg="unk_nav_sim" type="grid_node" output="screen">
    <rosparam file="$(find unk_nav_sim)/config/grid_params.yaml" command="load"/>
  </node>

  <!-- ══════════════════════════════════════════════════════════ -->
  <!-- 5. 导航节点                                                 -->
  <!-- ══════════════════════════════════════════════════════════ -->
  <node name="nav_node" pkg="unk_nav_sim" type="nav_node" output="screen">
    <param name="config_file" value="{cfg}"/>
  </node>

  <!-- ══════════════════════════════════════════════════════════ -->
  <!-- 6. RViz                                                     -->
  <!-- ══════════════════════════════════════════════════════════ -->
  <node name="rviz" pkg="rviz" type="rviz"
        args="-d $(find unk_nav_sim)/config/unk_nav.rviz"
        required="false"/>

</launch>
"""
    if not writable_by_gen(out_path):
        print(f"[launch] 跳过 {os.path.basename(out_path)}：已存在且非脚本生成"
              f"（保住手改内容；要重新生成请先删掉它）", file=sys.stderr)
        return False
    with open(out_path, "w") as f:
        f.write(content)
    print(f"wrote {os.path.relpath(out_path, os.getcwd())}")
    return True


def resolve_spawn(preset, nodes):
    """preset 里 spawn（显式 xy）与 spawn_s（按弧长取样）二选一；后者优先。"""
    if 'spawn_s' in preset:
        return sample(nodes, preset['spawn_s'])
    return tuple(preset['spawn'])


# ── 组装 world ────────────────────────────────────────────────────────

def world_header(name, tag, total, sp, close_err, lines, desc):
    sx, sy, syaw = sp
    road_launch = ('loop_road.launch' if name == 'loop'
                   else f'loop_{tag}_road.launch')
    goal_launch = ('loop_goal.launch' if name == 'loop'
                   else f'loop_{tag}_goal.launch')
    return f"""<?xml version="1.0" ?>
<!--
  {GEN_MARK} scripts/gen_road_world.py —— 请勿手改本文件；
  改道路形状请编辑脚本顶部 PRESETS['{name}'] 后重跑：
      python3 scripts/gen_road_world.py -p {name}
  （注：XML 注释内不得出现连续两短横，故此处用单短横别名 -p）

  {name}：首尾闭合的纯道路世界（无障碍）。
  {desc}
{os.linesep.join('  ' + t for t in lines)}
  中心线数值积分闭合残差 {close_err:.3f}m（设计为点对称闭合）。
  出生点 ({sx:.1f}, {sy:.1f}) 朝 yaw={syaw:.2f}rad。

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

{ground_plane()}"""


def build_one(name, args):
    """单个纯道路 preset：积分 → 校验 → 报告 → world → 两套 launch。"""
    preset = PRESETS[name]
    sections = preset['sections']
    prof = Corridor(preset['width'], preset.get('bias'))
    pairs, end = centerline_segments(sections, preset['ds'])
    if not validate(pairs, end, prof):
        print(f"[{name}] 存在几何问题，仍写出文件供调试，请修正参数后重跑",
              file=sys.stderr)
    nodes, total = polyline(pairs)
    close_err = math.hypot(end[0], end[1])
    sp = resolve_spawn(preset, nodes)
    lines = describe(prof, sections, nodes, total)
    print(f"\n{'=' * 74}\n预设 {name}：" + lines[0])
    print("\n".join("  " + t for t in lines[1:]))
    print(f"  出生点 ({sp[0]:.2f}, {sp[1]:.2f}) yaw={sp[2]:.2f}rad")
    xs = [p[0][0] for p in pairs] + [p[1][0] for p in pairs]
    ys = [p[0][1] for p in pairs] + [p[1][1] for p in pairs]
    print(f"  占地 x {min(xs):.1f}~{max(xs):.1f}m、y {min(ys):.1f}~{max(ys):.1f}m"
          f"（地面 {GROUND_SIZE:.0f}m）")
    if not args.no_ascii:
        print_ascii(nodes, total, prof, (), sp, name)
    if args.ascii:
        return
    tag = 'road' if name == 'loop' else name
    out_world = args.out or os.path.join(WORLD_DIR, f"loop_{tag}.world")
    os.makedirs(os.path.dirname(out_world), exist_ok=True)
    body = road_body(pairs, nodes, prof)
    with open(out_world, "w") as f:
        f.write(world_header(name, tag, total, sp, close_err, lines,
                             preset.get('desc', '')) + "".join(body) +
                "  </world>\n</sdf>\n")
    print(f"wrote {os.path.relpath(out_world, os.getcwd())}  "
          f"({len(pairs)} 段 × 3 = {len(pairs) * 3} 个 box)")
    check_xml(out_world)
    if args.no_launch:
        return
    stem = "loop" if name == 'loop' else f"loop_{tag}"
    made = 0
    for mode in ('road', 'goal'):
        lp = os.path.join(LAUNCH_DIR, f"{stem}_{mode}.launch")
        if build_launch(name, sp, out_world, lp, mode,
                        preset.get('launch_notes', [])):
            check_xml(lp)
            made += 1
    print(f"[{name}] world 已写出；launch 新写/更新 {made} 个"
          f"{'（其余为手改文件，已跳过）' if made < 2 else ''}，"
          f"XML well-formed 检查通过")


# 各场景往 launch 头部塞的实测告警（比 world 注释更要命，放这儿集中维护）
LAUNCH_NOTES = {
    'stadium': ["基线回归场：曲率沿半圆恒定、掉头是连续 180°，几何最简单。",
                "改跟路/控制参数后先跑它 —— 这里都不稳定就不是场景难，是实现退化了。"],
    'taper': ["变路宽：8 ↔ 3.2m 渐变收放各一次，最窄处净通行 1.8m（车侧 0.44m）。",
              "再窄会被 inflation 直接封死；收口斜率受 MAX_LAT_STEP 限制以防漏激光。",
              "看点：收窄段两侧路缘同时向中线靠拢，前瞻 7m 内的可通行断面持续变化。"],
    'asymturn': ["两侧特性不一样：大圆弧弯（R12，两侧近乎平行）与近直角弯并存。",
                 "直角弯处偏置 -1.5m → 左缘半径 1.2m（近直角）、右缘半径 7.2m（大圆弧），",
                 "同一次转向两侧曲率差 6 倍。按「走廊几何中线」跑的实现会在直角处吃内缘。"],
}
for _k, _v in LAUNCH_NOTES.items():
    PRESETS[_k]['desc'] = _v[0]
    PRESETS[_k]['launch_notes'] = _v


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="参数化生成闭合环形道路 world + launch")
    ap.add_argument("-p", "--preset", choices=sorted(PRESETS), default=None,
                    help="场景预设（默认 None = 生成全部）")
    ap.add_argument("-o", "--out", default=None,
                    help="输出 .world 路径（仅单 preset 时可用；默认 "
                         "../src/unk_nav_sim/worlds/loop_<tag>.world）")
    ap.add_argument("--ascii", action="store_true", help="只打印俯视示意图，不写文件")
    ap.add_argument("--no-ascii", action="store_true", help="不打印俯视示意图")
    ap.add_argument("--no-launch", action="store_true", help="只生成 world，不出 launch")
    ap.add_argument("--force-launch", action="store_true",
                    help="连没有 AUTO-GENERATED 标记的 launch 也重写（会吃掉手改内容，"
                         "确认那文件本来就是旧版脚本生成的再用）")
    args = ap.parse_args()
    FORCE_LAUNCH = args.force_launch
    names = [args.preset] if args.preset else sorted(PRESETS)
    if args.out and len(names) > 1:
        print("-o 仅单 preset 时可用", file=sys.stderr)
        sys.exit(2)
    for n in names:
        build_one(n, args)
