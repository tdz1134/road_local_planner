#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_obstacle_world.py —— 在闭合道路走廊内布设静态障碍，生成带障碍的 world + launch。

与 gen_road_world.py 的分工：道路骨架（中心线航向积分、路缘 box、三项几何校验）
直接 import 复用，本脚本只加「障碍层」与 launch 输出，改道路算法仍只需改一处。
（走廊生成循环有 ~12 行与 gen_road_world 重复：那是刻意的，为了不为了复用去改动
 一份已在用的脚本。出现第三份场景时再把该循环提取成公共函数。）

障碍用 Frenet 坐标定义：s = 沿中心线弧长 m，lat = 相对中心线横向偏移 m（左正）。
好处是障碍跟着路走 —— 改道路形状不需要重算 xy，且「这个断面留了多宽的口」能直接
从 lat 与尺寸算出来并打印，不用反推坐标。

为什么必须盯净宽（脚本对每个断面都算并分档）：
  · nav_params.yaml 的 inflation_radius = 0.7 → 两障碍间净宽低于 2×0.7 = 1.4m 时，
    膨胀后不存在 free 格链，A* 直接无路（症状：反复 RECOVERY 直到 ABORT，不硬闯）。
  · 净宽 ≥1.4m 也只保证「路径中心线能过」：A* 中心线距障碍 ≥0.7 而车半径 0.46，
    故车身侧向净空 = 净宽/2 - 0.46。1.4m 的口只剩 0.24m，这才是真临界点。
  · 分档：封死 <1.4 ≤ 临界 <1.8 ≤ 紧 <2.6 ≤ 宽松。

障碍高 1.5m，落在 grid_params.yaml 的 z_min 0.1 ~ z_max 2.5 之间 → 32 线激光有多条
回波，栅格里是实心块，不会因扫描线高度漏检。

用法：
    python3 scripts/gen_obstacle_world.py              # 生成全部预设（world + launch）
    python3 scripts/gen_obstacle_world.py -p tight     # 只生成 tight
    python3 scripts/gen_obstacle_world.py --ascii      # 只看俯视示意图，不写文件
    python3 scripts/gen_obstacle_world.py --no-launch  # 只出 world
"""

import argparse
import math
import os
import random
import sys
import xml.parsers.expat

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_road_world as rg  # noqa: E402  道路骨架复用

# ══════════════════ 与 nav_params.yaml / grid_params.yaml 保持同步 ══════════
INFLATION = 0.7       # inflation_radius：膨胀半径 m
ROBOT_R = 0.46        # robot_radius：车体半径 m
SEAL = 2.0 * INFLATION    # 1.4m：低于此该口没有 free 格链，A* 无路
TIGHT = 1.8           # 临界档上界：车侧净空 <=0.44m
LOOSE = 2.6           # 宽松档下界：车侧净空 >=0.84m
OBST_H = 1.5          # 障碍高 m（z 0~1.5，激光稳打）
# ═══════════════════════════════════════════════════════════════════════════

# 障碍种类（全部 static）：
#   post  单圆柱        s lat r
#   gate  对置双圆柱    s gap r          —— 中央留 gap 宽的口，两侧口由路缘封掉
#   box   斜置长方体    s lat half_l half_w yaw_deg
#   field 散布柱阵      s0 s1 n r seed   —— 固定 seed，可复现
#   block 横贯全封锁    s                —— 预期 RECOVERY→ABORT 的验证点
# 每项可带 why：会打印到断面清单里，说明这个断面在测什么。
#
# s 必须落在直段上：弯道与障碍同时改变两个变量，撞墙/绕不开时分不清是谁的锅。
# 曲率半径 14m 以上的缓弯可放行内偏移，但本预设一律避开。
PRESETS = {
    # 常规绕障：路宽 6m，每个断面都留有可绕口 → 车应绕过后回到路上并跑完整圈。
    'mid': {
        'width': 6.0, 'ds': 0.40, 'spawn_s': 2.0,
        # 只写前半：点对称闭合要求「前半净转角 = 180°」，重复一遍即自动闭合。
        # 注意 gen_road_world 是按 sections 原样积分的，不会替你先重复。
        'sections': [('line', 26.0), ('arc', 12.0, 45.0),
                     ('line', 8.0), ('arc', 5.0, 135.0)] * 2,
        'obstacles': [
            {'kind': 'post', 's': 10.0, 'lat': 0.0, 'r': 0.5,
             'why': '居中单柱：左右各 2.5m 都能过，测最基本的绕行与提前减速'},
            {'kind': 'post', 's': 18.0, 'lat': 1.2, 'r': 0.6,
             'why': '偏心柱：右侧口 1.2m 封死，只有左侧 3.6m 可过 → 非对称选择'},
            {'kind': 'gate', 's': 24.0, 'gap': 2.4, 'r': 0.5,
             'why': '双柱窄门：两侧口被路缘与柱夹成 0.8m 封死，必走中央 2.4m'},
            {'kind': 'field', 's0': 36.0, 's1': 42.9, 'n': 6, 'r': 0.35, 'seed': 11,
             'why': '随机柱阵（seed 固定）：测多障碍下 A* 迭代量与子目标扇形截断'},
            {'kind': 'post', 's': 66.0, 'lat': -1.0, 'r': 0.5,
             'why': '镜像偏心柱（半环另一侧）：确认左右偏好不是同一处几何的巧合'},
            {'kind': 'box', 's': 73.0, 'lat': 1.0, 'half_l': 1.2, 'half_w': 0.3,
             'yaw_deg': 25.0,
             'why': '斜置长块：非轴对齐形状 → 栅格阶梯化 + 两侧口不等，只能左绕'},
            {'kind': 'gate', 's': 79.5, 'gap': 1.9, 'r': 0.5,
             'why': '紧窄门 1.9m（车侧净空 0.49m）：绕得过去但贴边，测横向误差上限'},
            {'kind': 'field', 's0': 91.5, 's1': 97.9, 'n': 5, 'r': 0.4, 'seed': 5,
             'why': '第二处柱阵，换 seed 换疏密：看绕行路径是否稳定而非随机摆头'},
        ],
    },
    # 临界与封死：路宽压到 3.6m（inflation 0.7 → 净通行 2.2m ≈ 车宽 0.92 + 每侧 0.64）。
    # 前 4 个断面都是「理论上能过、实际只剩 0.2~0.3m」，最后一个是硬封死。
    'tight': {
        'width': 3.6, 'ds': 0.25, 'spawn_s': 2.0,
        'sections': [('line', 20.0), ('arc', 8.0, 45.0),
                     ('line', 6.0), ('arc', 3.5, 135.0)] * 2,
        'obstacles': [
            {'kind': 'post', 's': 9.0, 'lat': 0.0, 'r': 0.35,
             'why': '居中细柱：左右各 1.45m 刚好高于 1.4 封锁线 → 走哪边都行，测抖动'},
            {'kind': 'gate', 's': 16.0, 'gap': 1.6, 'r': 0.5,
             'why': '窄门 1.6m（车侧净空 0.34m）：两侧完全封死，只有中央一条 free 链'},
            {'kind': 'box', 's': 30.5, 'lat': -0.35, 'half_l': 0.9, 'half_w': 0.3,
             'yaw_deg': 30.0,
             'why': '斜块偏左：左口 0.74m 封死，右口 1.44m 临界 → 逼一次贴墙绕行'},
            {'kind': 'post', 's': 52.0, 'lat': 0.9, 'r': 0.45,
             'why': '偏心柱：右口 0.45m 封死，左口 2.25m 紧 → 单侧通行'},
            {'kind': 'block', 's': 58.5,
             'why': '横贯全宽的封锁墙：无路可绕。预期减速停住 → RECOVERY 重试 → ABORT。'
                    '放最后一个断面，否则它后面的用例全测不到'},
        ],
    },
}

WORLD_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "..", "src", "unk_nav_sim", "worlds")
LAUNCH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "..", "src", "unk_nav_sim", "launch")


# ── 中心线：由 seg_pairs 建弧长表并按弧长取样 ─────────────────────────

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


# ── 障碍展开为原子体 ───────────────────────────────────────────────────

def expand(obstacles, half):
    """把预设里的 gate/field/block 展开成单个体，便于统一算横向占位与净宽。"""
    out = []
    for spec in obstacles:
        kind, why = spec['kind'], spec.get('why', '')
        if kind == 'post':
            out.append(dict(shape='cyl', s=spec['s'], lat=spec['lat'],
                            r=spec['r'], why=why))
        elif kind == 'gate':
            c = 0.5 * spec['gap'] + spec['r']      # 柱心横向位置：让内侧边缘正好留出 gap
            for sgn in (-1.0, 1.0):
                out.append(dict(shape='cyl', s=spec['s'], lat=sgn * c,
                                r=spec['r'], why=why))
        elif kind == 'box':
            out.append(dict(shape='box', s=spec['s'], lat=spec['lat'],
                            half_l=spec['half_l'], half_w=spec['half_w'],
                            yaw=math.radians(spec.get('yaw_deg', 0.0)), why=why))
        elif kind == 'block':
            # 横贯全宽且两端各伸进路缘 0.25m：不留激光可穿的薄缝。span_full 告知
            # 越界校验「压进路缘」是故意的，不是配置错误
            out.append(dict(shape='box', s=spec['s'], lat=0.0, half_l=0.30,
                            half_w=half + 0.25, yaw=0.0, why=why, span_full=True))
        elif kind == 'field':
            rng = random.Random(spec['seed'])
            lat_max = half - spec['r'] - 0.1       # 保证柱体完整落在走廊内
            for _ in range(spec['n']):
                out.append(dict(shape='cyl',
                                s=spec['s0'] + (spec['s1'] - spec['s0']) * rng.random(),
                                lat=(-1.0 + 2.0 * rng.random()) * lat_max,
                                r=spec['r'], why=why))
        else:
            raise ValueError(f"未知障碍类型: {kind}")
    return out


def lat_half(ob):
    """障碍在走廊横向（lat 方向）上的半占位：斜置 box 取两半轴投影之和。"""
    if ob['shape'] == 'cyl':
        return ob['r']
    return abs(ob['half_w'] * math.cos(ob['yaw'])) + \
        abs(ob['half_l'] * math.sin(ob['yaw']))


def along_half(ob):
    """障碍沿道路方向的半占位，用于判断体是否伸进了弯道。"""
    if ob['shape'] == 'cyl':
        return ob['r']
    return abs(ob['half_l'] * math.cos(ob['yaw'])) + \
        abs(ob['half_w'] * math.sin(ob['yaw']))


def arc_ranges(nodes, tol=1e-9):
    """中心线上航向有变化的 s 区间 = 弯道。把「障碍只放直段」从注释升级成校验。"""
    out, cur = [], None
    for i in range(1, len(nodes)):
        curved = abs(nodes[i][3] - nodes[i - 1][3]) > tol
        if curved and cur is None:
            cur = nodes[i - 1][0]
        elif not curved and cur is not None:
            out.append((cur, nodes[i - 1][0]))
            cur = None
    if cur is not None:
        out.append((cur, nodes[-1][0]))
    return out


def cross_sections(obs, half, merge_ds=2.0):
    """
    按 s 聚成断面（|Δs| <= merge_ds 视为同一横断面），算每个断面的可通行口。
    返回 [(s代表, [(口宽, 左边界, 右边界), ...], why, 是否越界)]，口宽升序。
    """
    if not obs:
        return []
    items = sorted(obs, key=lambda o: o['s'])
    groups, cur = [], [items[0]]
    for ob in items[1:]:
        if ob['s'] - cur[-1]['s'] <= merge_ds:
            cur.append(ob)
        else:
            groups.append(cur)
            cur = [ob]
    groups.append(cur)

    out = []
    for g in groups:
        ivs = []
        why, oob = '', []
        for ob in g:
            lo, hi = ob['lat'] - lat_half(ob), ob['lat'] + lat_half(ob)
            ivs.append((lo, hi))
            why = why or ob['why']
            if (lo < -half or hi > half) and not ob.get('span_full'):
                oob.append(round(ob['s'], 1))
        ivs.sort()
        merged = []
        for lo, hi in ivs:
            if merged and lo <= merged[-1][1]:
                merged[-1] = (merged[-1][0], max(merged[-1][1], hi))
            else:
                merged.append((lo, hi))
        gaps, prev = [], -half
        for lo, hi in merged:
            if lo > prev:
                gaps.append((lo - prev, prev, lo))
            prev = hi
        if prev < half:
            gaps.append((half - prev, prev, half))
        gaps.sort(key=lambda t: t[0])
        out.append((sum(ob['s'] for ob in g) / len(g), gaps, why, sorted(set(oob))))
    return out


def verdict(w):
    if w < 2.0 * ROBOT_R:
        return '必挂'        # 连车宽都不够，即使关掉膨胀也过不去
    if w < SEAL:
        return '封死'        # 膨胀后无 free 链
    if w < TIGHT:
        return '临界'
    if w < LOOSE:
        return '紧'
    return '宽松'


# ── SDF 片段 ──────────────────────────────────────────────────────────

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


def road_body(pairs, width):
    """沥青（visual-only）+ 左右路缘墙。与 gen_road_world 同一套参数，接缝压合。"""
    curb_off = 0.5 * width + rg.CURB_T / 2.0
    body = []
    for i, ((x0, y0, t0), (x1, y1, t1)) in enumerate(pairs):
        body.append(rg.segment_box(f"asphalt_{i}", x0, y0, x1, y1,
                                   width, 0.01, 0.005, False, "Black", rg.JOINT_EXT))
        for side, tag in ((curb_off, "curbL"), (-curb_off, "curbR")):
            ax, ay = rg.offset_pt(x0, y0, t0, side)
            bx, by = rg.offset_pt(x1, y1, t1, side)
            body.append(rg.segment_box(f"{tag}_{i}", ax, ay, bx, by, rg.CURB_T,
                                       rg.CURB_H, rg.CURB_H / 2.0, True, "Grey",
                                       rg.JOINT_EXT))
    return body


def obstacle_models(obs, nodes):
    body = []
    for i, ob in enumerate(obs):
        x, y, th = sample(nodes, ob['s'])
        cx, cy = rg.offset_pt(x, y, th, ob['lat'])
        color = "Red" if ob['shape'] == 'cyl' else "Orange"
        if ob['shape'] == 'cyl':
            body.append(cylinder_model(f"obs_{i}", cx, cy, ob['r'], OBST_H,
                                       OBST_H / 2.0, color))
        else:
            body.append(rg.box_model(f"obs_{i}", cx, cy, th + ob['yaw'],
                                     2 * ob['half_l'], 2 * ob['half_w'], OBST_H,
                                     OBST_H / 2.0, True, color))
    return body


# ── 报告 ──────────────────────────────────────────────────────────────

def report(name, preset, total, obs, groups, spawn):
    width = preset['width']
    print(f"\n{'=' * 74}\n预设 {name}：路宽 {width:.1f}m，环长 {total:.1f}m，"
          f"障碍 {len(obs)} 个体 / {len(groups)} 个断面")
    print(f"出生点 s={preset['spawn_s']:.1f} → ({spawn[0]:.2f}, {spawn[1]:.2f}) "
          f"yaw={spawn[2]:.2f}rad（由中心线取样算出，保证落在路上）")
    print(f"膨胀 {INFLATION}m：口宽低于 {SEAL:.1f}m 即无 free 链 → A* 无路\n")
    print(f"{'s':>6} {'口数':>4}  各口宽度与档位（车侧净空 = 口宽/2 - "
          f"{ROBOT_R:.2f}）")
    sealed_all = []
    for s, gaps, why, oob in groups:
        if not gaps:
            cells = "全封死"
            sealed_all.append(round(s, 1))
        else:
            cells = "  ".join(f"{w:.2f}m[{verdict(w)}|净空{max(0.0, w / 2 - ROBOT_R):.2f}]"
                              for w, _, _ in gaps)
            if all(w < SEAL for w, _, _ in gaps):
                sealed_all.append(round(s, 1))
        print(f"{s:6.1f} {len(gaps):4}   {cells}")
        if why:
            print(f"{'':12}→ {why}")
        if oob:
            print(f"{'':12}⚠ s={oob} 处障碍伸出走廊，与路缘重叠")
    if sealed_all:
        print(f"\n完全封死断面：{sealed_all} → 预期在此减速停住、RECOVERY 重试后 ABORT")
    print("提示：弯道上不放障碍（曲率与障碍同时改变两个变量，撞墙时分不清谁的锅）")


# ── 俯视示意图 ────────────────────────────────────────────────────────

def render_ascii(nodes, total, obs, half, spawn=None, dx=0.5, dy=1.0, step=0.25):
    """终端字符约 2:1，故 dx = dy/2。step 必须明显小于 dx，否则斜段会漏格。"""
    pts, boxes, circles = [], [], []
    for s in [i * step for i in range(int(total / step) + 1)]:
        pts.append(sample(nodes, s))
    for ob in obs:
        x, y, th = sample(nodes, ob['s'])
        cx, cy = rg.offset_pt(x, y, th, ob['lat'])
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
    allx = [p[0] for p in pts] + [c[0] for b in boxes for c in b] + \
           [c[0] - c[2] for c in circles] + [c[0] + c[2] for c in circles]
    ally = [p[1] for p in pts] + [c[1] for b in boxes for c in b] + \
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

    for (x, y, th) in pts:
        for k in (-half, half):
            bx, by = rg.offset_pt(x, y, th, k)
            put(bx, by, '#')
        n = max(1, int(2 * half / 0.25))
        for j in range(n + 1):
            lx, ly = rg.offset_pt(x, y, th, -half + 2 * half * j / n)
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


def print_ascii(nodes, total, obs, half, spawn):
    grid, nh, nw = render_ascii(nodes, total, obs, half, spawn=spawn)
    print(f"\n俯视示意  {nw}×{nh} 格（横 0.5m/格、纵 1.0m/格）"
          f"      . 走廊   # 路缘   O 障碍   S 出生点")
    print("\n".join(''.join(r).rstrip() for r in grid))


# ── 组装 world / launch ───────────────────────────────────────────────

def ground_plane():
    return f"""    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
          <geometry><plane><normal>0 0 1</normal>
            <size>{rg.GROUND_SIZE:.0f} {rg.GROUND_SIZE:.0f}</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal>
            <size>{rg.GROUND_SIZE:.0f} {rg.GROUND_SIZE:.0f}</size></plane></geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Green</name></script></material>
        </visual>
      </link>
    </model>

"""


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


def build_launch(name, preset, spawn, world_file, out_path, mode):
    """mode='road' 沿路（无定位）；mode='goal' 终点导航（有定位，RViz 点目标）。"""
    sx, sy, syaw = spawn
    world_base = os.path.basename(world_file)
    if mode == 'road':
        cfg = "$(find unk_nav)/config/nav_params_road.yaml"
        loc = ('  <node name="localization_node" pkg="unk_nav_sim" '
               'type="localization_node"\n'
               '        output="screen" if="$(arg use_localization)">\n'
               '    <param name="model_name" value="scout/"/>\n'
               '  </node>\n')
        extra = '  <arg name="use_localization" default="false"/>\n'
        fixed = "base_link"
        blurb = ("沿路模式（follow_road=true）：NavCore 从路缘走廊几何推车体系前瞻子目标，\n"
                 "        不读定位与全局终点；障碍由下游 A* 负责绕行，绕不过就如实 RECOVERY/ABORT。")
    else:
        cfg = "$(find unk_nav)/config/nav_params.yaml"
        loc = ('  <node name="localization_node" pkg="unk_nav_sim" '
               'type="localization_node"\n'
               '        output="screen">\n'
               '    <param name="model_name" value="scout/"/>\n'
               '  </node>\n')
        extra = ''
        fixed = "odom"
        blurb = ("终点模式（follow_road=false）：RViz 点 2D Nav Goal，障碍绕行是纯 A* 行为。\n"
                 "        注意终点要挑在走廊内且距障碍不小于 goal_clear_radius，否则斜穿草地。")
    content = f"""<?xml version="1.0"?>
<!--
  {os.path.basename(out_path)}：{blurb}
  一键启动：Gazebo({world_base}) + Scout v2 + 栅格 + 导航 + RViz

  世界由 scripts/gen_obstacle_world.py 生成：走廊内带静态障碍的闭合道路，
  路宽 {preset['width']:.1f}m，出生点见下方 arg。障碍全 static，含若干「口宽刚好高于
  膨胀封锁线 {SEAL:.1f}m」的临界断面，以及（tight）一个完全封死断面。

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
    with open(out_path, "w") as f:
        f.write(content)
    print(f"wrote {os.path.relpath(out_path, os.getcwd())}")


# ── 主流程 ────────────────────────────────────────────────────────────

def build_one(name, args):
    preset = PRESETS[name]
    pairs, end = rg.centerline_segments(preset['sections'], preset['ds'])
    if not rg.validate(pairs, end, preset['width']):
        print(f"[{name}] 几何校验未通过，仍写出文件供调试", file=sys.stderr)
    nodes, total = polyline(pairs)
    half = 0.5 * preset['width']
    obs = expand(preset['obstacles'], half)
    groups = cross_sections(obs, half)
    arcs = arc_ranges(nodes)
    intrude = sorted({round(o['s'], 1) for o in obs
                      for lo, hi in arcs
                      if o['s'] + along_half(o) > lo and o['s'] - along_half(o) < hi})
    sp = sample(nodes, preset['spawn_s'])

    report(name, preset, total, obs, groups, sp)
    if intrude:
        print(f"⚠ 障碍体伸进弯道（约定只放直段）：s={intrude}\n"
              f"  弯道 s 区间：{[(round(a, 1), round(b, 1)) for a, b in arcs]}",
              file=sys.stderr)
    if not args.no_ascii:
        print_ascii(nodes, total, obs, half, sp)
    if args.ascii:
        return
    out_world = args.out or os.path.join(WORLD_DIR, f"obstacle_{name}.world")
    os.makedirs(os.path.dirname(out_world), exist_ok=True)
    write_world(name, preset, pairs, nodes, total, obs, groups, sp, out_world)
    check_xml(out_world)
    if not args.no_launch:
        for mode in ('road', 'goal'):
            lp = os.path.join(LAUNCH_DIR, f"obstacle_{name}_{mode}.launch")
            build_launch(name, preset, sp, out_world, lp, mode)
            check_xml(lp)
    print(f"[{name}] world 与 launch 已写出，XML well-formed 检查通过")


def write_world(name, preset, pairs, nodes, total, obs, groups, sp, out_path):
    width = preset['width']
    lines = []
    for s, gaps, _why, _oob in groups:
        gs = " ".join(f"{w:.2f}m[{verdict(w)}]" for w, _, _ in gaps) or "全封死"
        lines.append(f"s={s:5.1f}m  {gs}")
    body = road_body(pairs, width) + obstacle_models(obs, nodes)
    header = f"""<?xml version="1.0" ?>
<!--
  AUTO-GENERATED by scripts/gen_obstacle_world.py —— 请勿手改本文件；
  改场景请编辑脚本顶部 PRESETS['{name}'] 的 obstacles 列表后重跑：
      python3 scripts/gen_obstacle_world.py -p {name}
  （注：XML 注释内不得出现连续两短横，故此处用单短横别名 -p）

  {name}：走廊内带静态障碍的闭合道路。
  路宽 {width:.1f}m，配 inflation_radius {INFLATION} → 无障碍段净通行 {width - 2 * INFLATION:.1f}m。
  障碍 {len(obs)} 个体，分布在 {len(groups)} 个横断面；全部 static，高 {OBST_H}m
  （在 grid z_min 0.1 与 z_max 2.5 之间，激光稳打）。

  各断面可通行口宽（口宽低于 {SEAL:.1f}m 则膨胀后无 free 链，A* 无路）：
{os.linesep.join('      ' + t for t in lines) if lines else '      无'}

  出生点 x {sp[0]:.2f} y {sp[1]:.2f} yaw {sp[2]:.2f}，由中心线 s={preset['spawn_s']:.1f}m 取样得到，
  必然落在路上；第一段是 {preset['sections'][0][1]:.0f}m 直道，障碍从更远处开始。
-->
<sdf version="1.6">
  <world name="obstacle_{name}">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <include><uri>model://sun</uri></include>

{ground_plane()}"""
    with open(out_path, "w") as f:
        f.write(header + "".join(body) + "  </world>\n</sdf>\n")
    print(f"wrote {os.path.relpath(out_path, os.getcwd())}  "
          f"（道路 {len(pairs) * 3} box + 障碍 {len(obs)} 个体）")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="生成走廊内带静态障碍的闭合道路 world + launch")
    ap.add_argument("-p", "--preset", choices=sorted(PRESETS), default=None,
                    help="场景预设（默认 None = 生成全部）")
    ap.add_argument("-o", "--out", default=None, help="输出 .world 路径（仅单 preset）")
    ap.add_argument("--ascii", action="store_true", help="只打印俯视示意图，不写文件")
    ap.add_argument("--no-ascii", action="store_true", help="不打印俯视示意图")
    ap.add_argument("--no-launch", action="store_true", help="只生成 world，不出 launch")
    args = ap.parse_args()
    names = [args.preset] if args.preset else sorted(PRESETS)
    if args.out and len(names) > 1:
        print("-o 仅单 preset 时可用", file=sys.stderr)
        sys.exit(2)
    for n in names:
        build_one(n, args)
