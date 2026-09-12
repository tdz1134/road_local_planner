#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_obstacle_world.py —— 在闭合道路走廊内布设静态障碍，生成带障碍的 world + launch。

与 gen_road_world.py 的分工：道路骨架（中心线航向积分、路缘 box、六项几何校验、
world/launch 组装、俯视示意图）全部在那边，本脚本只加「障碍层」：
Frenet 定义 → 展开成原子体 → 逐断面算净宽 → 约束校验 → 写 world。
（障碍 preset 可以用 'road': '<道路preset名>' 直接继承同名纯道路场景，保证
 「有障碍 / 无障碍」两个 world 的道路部分全等，出问题好归因。）

障碍用 Frenet 坐标定义：s = 沿中心线弧长 m，lat = 相对中心线横向偏移 m（左正）。
好处是障碍跟着路走 —— 改道路形状不需要重算 xy，且「这个断面留了多宽的口」能直接
从 lat 与尺寸算出来并打印，不用反推坐标。路宽沿 s 变化（taper 场景）或有偏置
（asymturn 场景）时，走廊边界 left(s)/right(s) 不再是 ±半宽，脚本按 s 逐点取。

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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_road_world as rg  # noqa: E402  道路骨架 + world/launch 组装全部复用

# ══════════════════ 与 nav_params.yaml / grid_params.yaml 保持同步 ══════════
INFLATION = rg.INFLATION    # inflation_radius：膨胀半径 m（单一来源在 gen_road_world）
ROBOT_R = rg.ROBOT_R        # robot_radius：车体半径 m
SEAL = 2.0 * INFLATION      # 1.4m：低于此该口没有 free 格链，A* 无路
TIGHT = 1.8                 # 临界档上界：车侧净空 <=0.44m
LOOSE = 2.6                 # 宽松档下界：车侧净空 >=0.84m
OBST_H = 1.5                # 障碍高 m（z 0~1.5，激光稳打）
# ═══════════════════════════════════════════════════════════════════════════
# 障碍能不能放在弯道上的判据（按「体」分两类，理由不同，别混成一条「弯道一律不放」）：
#   · 直边体（box / wall / block）：SDF 里只有直线棱柱，弯道上「沿 s 平移」会扇形
#     展开或收拢 —— 7m 长的墙放在 R5 的弯上，中点与端点差出 1.2m，占位和断面报告
#     直接不成立 → 一律禁止，整段必须落在直道。
#   · 圆盘体（post / gate / row / field）：盘心落在该 s 的横断线上，横向夹口宽度与
#     笛卡尔严格相等（没有弓高误差），间距畸变只有 (Δθ)²/24 量级 → 几何上允许上
#     R ≥ MIN_R_FACTOR × 路宽 的缓弯。但「弯中绕障」会让撞墙归因分不清（是曲率的锅
#     还是障碍的锅？），所以还要作者显式写 ok_on_arc=True + 理由才放行。
#   · 直角弯（本仓库里 R2.7 那一类）：两类都禁 —— 直边体几何就不对，圆盘体则把
#     「转向还没结束就要横向避让」和「挤门洞」两件事叠在一起，出事故无法定位。
# 2.0 这个倍数怎么来的：R = 2× 路宽时，穿一个 2m 宽的门洞（横移约 1.5m、沿路 2m）
# 期间车头大约转 24° —— 转向与避让还能分开看。再陡（如 R5 配 6m 路，转 60°）就分不开了。
MIN_R_FACTOR = 2.0          # 圆盘体允许进弯的最小曲率半径 / 路宽 倍数

# 障碍种类（全部 static）：
#   post  单圆柱        s lat r
#   gate  对置双圆柱    s gap r          —— 中央留 gap 宽的口，两侧口由路缘封掉
#   box   斜置长方体    s lat half_l half_w yaw_deg
#   row   等间距柱阵    s0 s1 n r lat    —— 与 field 相对：疏密可预期，能算重复绕行
#   field 随机柱阵      s0 s1 n r seed   —— 固定 seed，可复现
#   wall  单侧内缩墙    s0 s1 face t [to_edge]
#                                    —— face = 朝路面那一侧到中心线的带符号距离；
#                                       to_edge=true 则墙体一直砌到路缘，不留激光能
#                                       钻的暗槽。用途：把走廊单侧压窄（另一侧不变），
#                                       造「两侧特性不一样」的第二种做法
#   block 横贯全封锁    s                —— 预期 RECOVERY→ABORT 的验证点
# 每项可带 why（打印到断面清单，说明这个断面在测什么）、ok_on_arc（显式允许进缓弯）。
#
# 弯道上放障碍的唯一破例方式：写 'ok_on_arc': True 并在 why 里说清为什么这处能承受
# —— 脚本按 MIN_R_FACTOR 判，不达标照样报错。
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
    # 变路宽 + 障碍：道路层直接继承 gen_road_world 的 'taper'（宽 7.0 ↔ 窄 3.2 渐变），
    # 所以这里的看点是「同一根柱子在不同宽度上的后果完全不同」—— 断面表会逐条把
    # 每个口的实际宽度打出来，宽处 2.2m 宽松、窄处 0.8m 封死，一目了然。
    # 刻意不在窄段（3.2m，s 14~22 / 67~75）放任何障碍：半宽才 1.6m，随便一根 r=0.4
    # 的柱子两侧就双双掉到 1.2m < SEAL 1.4m → 整环从此封死，后面的用例全测不到。
    # 收口/拓宽的斜坡上放障（s=8、36、62、88 附近）是本场景独有价值：无障碍版本里
    # 斜坡只改变路缘形状，这里还额外改变「绕障要横移多少」。
    'taper': {
        'road': 'taper', 'spawn_s': 2.0,
        'obstacles': [
            {'kind': 'post', 's': 8.0, 'lat': 0.0, 'r': 0.5,
             'why': '收口途中的居中柱（该处宽约 5.5m）：左右各 2.2m，和 mid 场景同一根'
                    '柱但口子明显小了，测「绕行余量随 s 收紧」'},
            {'kind': 'gate', 's': 36.0, 'gap': 2.4, 'r': 0.5,
             'why': '拓宽途中的窄门（该处宽约 5.2m）：两侧口被挤到 0.4m 封死，只能穿中央'},
            {'kind': 'post', 's': 55.0, 'lat': 1.2, 'r': 0.6,
             'why': '宽段（7m）偏心柱：与 s=8 那根同参，只差路宽 → 单变量对照'},
            {'kind': 'box', 's': 62.0, 'lat': 0.8, 'half_l': 1.0, 'half_w': 0.3,
             'yaw_deg': 20.0,
             'why': '第二次收口斜坡上的斜块：非轴对齐 + 走廊同时在收窄，'
                    '测两种约束叠加时 A* 是否还会选右侧'},
            {'kind': 'row', 's0': 88.0, 's1': 94.4, 'n': 3, 'r': 0.35, 'lat': 0.0,
             'why': '拓宽段等间距三柱（间距 3.2m）：同一根柱在 4.7m 和 7m 宽的断面上'
                    '后果不同，把「绕障横移量随路宽变化」这一件事单独打出来'},
        ],
    },
    # 两侧特性不一样 + 障碍：道路层继承 'asymturn'（大圆弧弯与近直角弯并存，直角弯处
    # 中心线被压向内一侧）。这里的看点是「走廊的单侧被改变」和「刚转完就要横向避让」。
    # 障碍布置顺序刻意贴着几何走：直道基线 → 直角弯前的单侧墙 → 直角弯出口柱 →
    # 缓弯中的窄门（唯一显式破例进弯道的用例）→ 后半镜像对照。
    'asymturn': {
        'road': 'asymturn', 'spawn_s': 3.0,
        'obstacles': [
            {'kind': 'post', 's': 12.0, 'lat': 1.2, 'r': 0.6,
             'why': '基线：与 mid 场景同一根偏心柱（右侧口封死，只左绕），'
                    '先确认这条路上最朴素的行为没变'},
            {'kind': 'wall', 's0': 33.0, 's1': 40.0, 'face': 1.8, 't': 0.3,
             'to_edge': True,
             'why': '直角弯入口前 1.4m 起，左半侧被连续内缩墙吃掉 1.2m：'
                    '走廊从 6m 压到 4.8m 且整体偏右，出弯前既要贴墙又要准备 90° 转向'},
            {'kind': 'post', 's': 48.0, 'lat': -1.0, 'r': 0.5,
             'why': '直角弯出口直道（仅 6m）正中偏右的柱子：转向余量还没清零就要立刻'
                    '横向避让，测「弯中/弯后立刻绕障」这一类真实路口工况'},
            {'kind': 'gate', 's': 26.7, 'gap': 2.0, 'r': 0.5, 'ok_on_arc': True,
             'why': '缓弯 R12（= 2× 路宽，脚本允许进弯的下限）中的窄门：横向夹口在径向上'
                    '与笛卡尔严格相等，破例只因为想测「弯中收窄」这一种工况'},
            {'kind': 'row', 's0': 64.0, 's1': 73.0, 'n': 3, 'r': 0.4, 'lat': 0.9,
             'why': '后半直道上一排靠右的桩（间距 4.5m）：三次重复同一个非对称绕行，'
                    '看选择是否稳定，而不是每根桩重新抖一次'},
            {'kind': 'gate', 's': 97.0, 'gap': 1.9, 'r': 0.5,
             'why': '第二个直角弯前最后一段直道上的紧窄门（车侧净空 0.49m）：'
                    '出窄门 4m 就是 90°，贴边误差会直接带到转弯里'},
        ],
    },
}

WORLD_DIR = rg.WORLD_DIR
LAUNCH_DIR = rg.LAUNCH_DIR


# ── 道路层来源：自带 sections 或继承 gen_road_world 的 preset ──────────

def road_of(preset):
    """
    返回 (sections, ds, Corridor)。给了 'road' 就用同名纯道路场景的几何，
    本地只写障碍 —— 这样「有障碍/无障碍」两个 world 的道路部分逐 box 全等。
    """
    src = dict(rg.PRESETS[preset['road']]) if 'road' in preset else {}
    src.update(preset)
    for k in ('sections', 'ds', 'width'):
        if k not in src:
            raise KeyError(f"障碍预设缺 '{k}'（也没继承到，'road' 名写错了？）")
    return src['sections'], src['ds'], rg.Corridor(src['width'], src.get('bias'))


# ── 障碍展开为原子体 ───────────────────────────────────────────────────

def expand(obstacles, prof, nodes):
    """
    把预设里的复合障碍展开成单个体，便于统一算横向占位与净宽。
    prof/nodes 用来按 s 取走廊左右边界（变宽、有偏置时两侧不再对称）。
    每个原子带 straight=True 表示它是「直边体」（box / wall / block）：形状由直线
    棱柱构成，在弯道上会扇形失真 —— 约束校验据此分两类判（见 MIN_R_FACTOR 处注释）。
    """
    out = []
    for spec in obstacles:
        kind, why = spec['kind'], spec.get('why', '')
        arc_ok = spec.get('ok_on_arc', False)
        common = dict(why=why, ok_on_arc=arc_ok)
        if kind == 'post':
            out.append(dict(shape='cyl', s=spec['s'], lat=spec['lat'],
                            r=spec['r'], straight=False, **common))
        elif kind == 'gate':
            c = 0.5 * spec['gap'] + spec['r']      # 柱心横向位置：让内侧边缘正好留出 gap
            for sgn in (-1.0, 1.0):
                out.append(dict(shape='cyl', s=spec['s'], lat=sgn * c,
                                r=spec['r'], straight=False, **common))
        elif kind == 'box':
            out.append(dict(shape='box', s=spec['s'], lat=spec['lat'],
                            half_l=spec['half_l'], half_w=spec['half_w'],
                            yaw=math.radians(spec.get('yaw_deg', 0.0)),
                            straight=True, **common))
        elif kind == 'block':
            # 横贯全宽且两端各伸进路缘 0.25m：不留激光可穿的薄缝。span_full 告知
            # 越界校验「压进路缘」是故意的，不是配置错误
            s = spec['s']
            wl, wr = prof.left(s), prof.right(s)
            out.append(dict(shape='box', s=s, lat=0.5 * (wl - wr),
                            half_l=0.30, half_w=0.5 * (wl + wr) + 0.25,
                            yaw=0.0, straight=True, why=why, ok_on_arc=arc_ok,
                            span_full=True))
        elif kind == 'wall':
            # 单侧内缩墙：face 是朝路面一侧到中心线的带符号距离（正=左侧墙），
            # 墙体往所属那侧砌出去。拆成 ~2m 一块：一是让断面分析还能按 s 分组，
            # 二是顺手让相邻块在接缝处各多伸 0.1m 压合，防激光从墙缝里漏过去。
            # to_edge=True 时一直砌到路缘 —— 不留「墙与路缘之间那条激光能钻的暗槽」
            face, t = spec['face'], spec.get('t', 0.3)
            s0, s1 = spec['s0'], spec['s1']
            if face == 0.0:
                raise ValueError("wall.face 不能是 0（那是把墙横在路中央，用 block）")
            n = max(1, int(round((s1 - s0) / 2.0)))
            step = (s1 - s0) / n
            for i in range(n):
                sc = s0 + step * (i + 0.5)
                wl, wr = prof.left(sc), prof.right(sc)
                outer = wl if face > 0 else -wr       # 该侧走廊边界
                if spec.get('to_edge'):
                    if abs(face) >= abs(outer):
                        raise ValueError(f"wall 在 s={sc:.1f} 处 face={face} 已在路缘"
                                         f"之外（该侧缘距 {outer:.2f}）")
                    lat_c, half_w = 0.5 * (face + outer), 0.5 * abs(outer - face)
                else:
                    lat_c = face + (t / 2.0 if face > 0 else -t / 2.0)
                    half_w = t / 2.0
                out.append(dict(shape='box', s=sc, lat=lat_c,
                                half_l=step / 2.0 + 0.1, half_w=half_w, yaw=0.0,
                                straight=True, why=why, ok_on_arc=arc_ok))
        elif kind in ('row', 'field'):
            s0, s1, n, r = spec['s0'], spec['s1'], spec['n'], spec['r']
            if kind == 'row':
                # 等间距（含两端）：疏密可预期，才能拿「同一动作重复 n 次」当观测量
                draws = [((s0 + (s1 - s0) * (i / (n - 1.0) if n > 1 else 0.5)),
                          spec['lat']) for i in range(n)]
            else:
                rng = random.Random(spec['seed'])         # 固定 seed，可复现
                draws = []
                for _ in range(n):
                    sc = s0 + (s1 - s0) * rng.random()
                    # 随机柱只能在走廊内：按该 s 处实际的左/右缘距取范围。
                    # 越界（face 给到路缘外）由 cross_sections 的 oob 检查报出来
                    v = rng.random()
                    a = -prof.right(sc) + r + 0.1
                    b = prof.left(sc) - r - 0.1
                    draws.append((sc, a + (b - a) * v))
            for sc, lat in draws:
                out.append(dict(shape='cyl', s=sc, lat=lat, r=r, straight=False,
                                cluster=True, **common))
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


# ── 弯道约束：两类原子两套判据 ─────────────────────────────────────────

def arc_ranges(nodes, tol=1e-9):
    """中心线上航向有变化的 s 区间 = 弯道。（跨 s 原子的禁区）"""
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


def curv_radius(nodes, s):
    """该 s 处中心线局部曲率半径（直线 → inf）。用相邻两节点的弦/航向差，偏保守。"""
    for i in range(len(nodes) - 1):
        dth = nodes[i + 1][3] - nodes[i][3]
        if abs(dth) < 1e-9:
            continue
        chord = math.hypot(nodes[i + 1][1] - nodes[i][1],
                           nodes[i + 1][2] - nodes[i][2])
        lo, hi = nodes[i][0], nodes[i + 1][0]
        if lo <= s <= hi + 1e-9:
            return chord / abs(dth)
    return float('inf')


def constraint_check(obs, prof, nodes):
    """
    返回违规说明列表（空 = 全部符合约定）。判据见文件头 MIN_R_FACTOR 处：
    直边体一点都不许碰弯道（含自身沿 s 的跨度），圆盘体只看盘心所在处是否够缓。
    """
    arcs = arc_ranges(nodes)
    bad = []
    for o in obs:
        ext = 0.0 if o['shape'] == 'cyl' else along_half(o)
        lo, hi = o['s'] - ext, o['s'] + ext
        on_arc = any(lo < b and hi > a for a, b in arcs)
        if not on_arc:
            continue
        r_loc = curv_radius(nodes, o['s'])
        if o['straight']:
            bad.append(f"s={o['s']:.1f} 直边体（box/墙/块）[{lo:.1f},{hi:.1f}] 压在弯道"
                       f"（局部 R={r_loc:.1f}m）：沿 s 平移在弯道上扇形展开，"
                       f"弓高误差 {(hi - lo) ** 2 / (8 * r_loc):.2f}m，"
                       f"占位与断面报告都不成立")
        elif r_loc < MIN_R_FACTOR * prof.width(o['s']):
            bad.append(f"s={o['s']:.1f} 圆盘体在 R={r_loc:.1f}m 的弯上，而该处路宽 "
                       f"{prof.width(o['s']):.1f}m（要求 R ≥ {MIN_R_FACTOR}× 宽）："
                       f"转弯与绕障两件事叠在一起，撞墙了分不清是谁的锅")
        elif not o['ok_on_arc']:
            bad.append(f"s={o['s']:.1f} 圆盘体在缓弯 R={r_loc:.1f}m 上（几何可接受），"
                       f"但没标 ok_on_arc=True：破例要写明理由")
    return sorted(set(bad))        # 同一 gate 的两根柱会说同一条话，去重


def cross_sections(obs, prof, nodes, merge_ds=2.0):
    """
    按 s 聚成断面（|Δs| <= merge_ds 视为同一横断面），算每个断面的可通行口。
    返回 [(s代表, s起, s止, [(口宽, 左边界, 右边界), ...], why, 越界s列表, 是否柱阵簇)]，
    口宽升序。走廊边界按 s 取：变宽/有偏置时左右不再对称。
    一个断面跨多个 s 时，取该组里「最窄」的那个 s 算边界（偏保守，报出来的口
    不会比实际更宽）。

    「柱阵簇」（row/field 里间距与柱径同量级）单独标出来：把 2m 内的柱全压到同一
    横断面上判封死，是明显偏保守的 —— 交错排列的柱子之间存在斜穿的自由链。这一列
    只作最不利参考，不参与「本场景会在哪里 ABORT」的结论（那必须由单一断面的
    post/gate/box/block 给出，它们的判定是严格的）。
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
        # 参考 s：组内最窄处（口径以它为准，报出来的口不宽于实际）
        s_ref = min((o['s'] for o in g), key=prof.width)
        lo_b, hi_b = -prof.right(s_ref), prof.left(s_ref)
        for ob in g:
            lo, hi = ob['lat'] - lat_half(ob), ob['lat'] + lat_half(ob)
            if (lo < lo_b or hi > hi_b) and not ob.get('span_full'):
                oob.append(round(ob['s'], 1))
        ivs.sort()
        merged = []
        for lo, hi in ivs:
            if merged and lo <= merged[-1][1]:
                merged[-1] = (merged[-1][0], max(merged[-1][1], hi))
            else:
                merged.append((lo, hi))
        gaps, prev = [], lo_b
        for lo, hi in merged:
            if lo > prev:
                gaps.append((lo - prev, prev, lo))
            prev = max(prev, hi)
        if prev < hi_b:
            gaps.append((hi_b - prev, prev, hi_b))
        gaps.sort(key=lambda t: t[0])
        dense = len(g) > 1 and any(o.get('cluster') for o in g)
        out.append((s_ref, g[0]['s'], g[-1]['s'], gaps, why, sorted(set(oob)),
                    dense))
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

def obstacle_models(obs, nodes):
    body = []
    for i, ob in enumerate(obs):
        x, y, th = rg.sample(nodes, ob['s'])
        cx, cy = rg.offset_pt(x, y, th, ob['lat'])
        color = "Red" if ob['shape'] == 'cyl' else "Orange"
        if ob['shape'] == 'cyl':
            body.append(rg.cylinder_model(f"obs_{i}", cx, cy, ob['r'], OBST_H,
                                          OBST_H / 2.0, color))
        else:
            body.append(rg.box_model(f"obs_{i}", cx, cy, th + ob['yaw'],
                                     2 * ob['half_l'], 2 * ob['half_w'], OBST_H,
                                     OBST_H / 2.0, True, color))
    return body


# ── 报告 ──────────────────────────────────────────────────────────────

def report(name, preset, sections, prof, total, obs, groups, spawn):
    wmin, wmax = prof.w.span(0.0, total)
    print(f"\n{'=' * 74}\n预设 {name}：路宽 {wmin:.1f}"
          + (f" ~ {wmax:.1f}m（变宽）" if wmin != wmax else "m，")
          + f"环长 {total:.1f}m，障碍 {len(obs)} 个体 / {len(groups)} 个断面")
    print(f"出生点 s={preset['spawn_s']:.1f} → ({spawn[0]:.2f}, {spawn[1]:.2f}) "
          f"yaw={spawn[2]:.2f}rad（由中心线取样算出，保证落在路上）")
    print(f"膨胀 {INFLATION}m：口宽低于 {SEAL:.1f}m 即无 free 链 → A* 无路\n")
    print(f"{'s':>6} {'口数':>4}  该处路宽   各口宽度与档位（车侧净空 = 口宽/2 - "
          f"{ROBOT_R:.2f}）")
    sealed_all, worst_cluster = [], []
    for s_ref, g0, g1, gaps, why, oob, dense in groups:
        if not gaps:
            cells = "全封死"
        else:
            cells = "  ".join(f"{w:.2f}m[{verdict(w)}|净空{max(0.0, w / 2 - ROBOT_R):.2f}]"
                              for w, _, _ in gaps)
        span = f"{g0:.1f}~{g1:.1f}" if g1 - g0 > 0.5 else f"{g0:.1f}"
        print(f"{span:>12} {len(gaps):4}   {prof.width(s_ref):5.1f}m   {cells}"
              + ("　†柱阵簇最不利断面" if dense else ""))
        if why:
            print(f"{'':14}→ {why}")
        if oob:
            print(f"{'':14}⚠ s={oob} 处障碍伸出走廊，与路缘重叠")
        if dense:
            worst_cluster.append(round(s_ref, 1))
        elif not gaps or all(w < SEAL for w, _, _ in gaps):
            sealed_all.append(round(s_ref, 1))
    if sealed_all:
        print(f"\n完全封死断面：{sealed_all} → 预期在此减速停住、RECOVERY 重试后 ABORT")
    if worst_cluster:
        print(f"† 柱阵簇（s={worst_cluster}）按「2m 内的柱压到同一断面」判，偏保守："
              f"交错柱之间有斜穿自由链，实际能不能过以仿真为准。"
              f"若车确实在这里停住，说明簇内间距给得不够，不是判据错了")
    print(f"提示：直边体（box/墙/块）整段必须在直道；圆盘体（柱/门/柱阵）可上 R ≥ "
          f"{MIN_R_FACTOR}× 路宽的缓弯，但须标 ok_on_arc 并写理由（见文件头判据注释）")


# ── 组装 world / launch ───────────────────────────────────────────────

OBSTACLE_NOTES = {
    'mid': ["常规绕障：每个断面都留有可绕口 → 车应绕过后回到路上并跑完整圈。",
            "看点是「绕完能不能回到走廊」，不是「能不能挤过去」。"],
    'tight': ["临界与封死：多个断面只剩 0.2~0.3m 车侧净空，最后一个断面硬封死。",
              "预期：封死处减速停住 → RECOVERY 重试 → ABORT（不是撞上去，是停住）。"],
    'taper': ["变路宽 + 障碍：道路层继承 gen_road_world 的 taper（7.0 ↔ 3.2m 渐变）。",
              "看点：同一根柱在宽处和窄处后果完全不同 —— 断面表逐条打实际口宽。",
              "窄段（3.2m）刻意不放障碍：半宽才 1.6m，一根 r=0.4 的柱就能把整环封死。"],
    'asymturn': ["两侧特性不一样 + 障碍：道路层继承 asymturn（大圆弧弯与近直角弯并存）。",
                 "直角弯前有单侧内缩墙把走廊压到 4.8m，出口 2.3m 处又有一根偏心柱：",
                 "「刚转完就要横向避让」是本场景独有一条。缓弯中的窄门是显式破例用例。"],
}


def world_header(name, sections, prof, pairs, obs, groups, sp, total, notes):
    lines = []
    for s_ref, g0, g1, gaps, _why, _oob, dense in groups:
        gs = " ".join(f"{w:.2f}m[{verdict(w)}]" for w, _, _ in gaps) or "全封死"
        tag = f"s={s_ref:5.1f}m" + (f"（{g0:.1f}~{g1:.1f}）" if g1 - g0 > 0.5 else "")
        note = "　（柱阵簇最不利断面，交错柱之间可斜穿）" if dense else ""
        lines.append(f"  {tag}: 该处路宽 {prof.width(s_ref):.1f}m → {gs}{note}")
    wmin, wmax = prof.w.span(0.0, total)
    spans, _ = rg.section_spans(sections)
    return f"""<?xml version="1.0" ?>
<!--
  {rg.GEN_MARK} scripts/gen_obstacle_world.py —— 请勿手改本文件；
  改场景请编辑脚本顶部 PRESETS['{name}'] 的 obstacles 列表后重跑：
      python3 scripts/gen_obstacle_world.py -p {name}
  （注：XML 注释内不得出现连续两短横，故此处用单短横别名 -p）

  {name}：走廊内带静态障碍的闭合道路。
  路宽 {wmin:.1f}{"" if wmin == wmax else f" ~ {wmax:.1f}"}m，配 inflation_radius {INFLATION} →
  最窄处净通行 {wmin - 2 * INFLATION:.1f}m。障碍 {len(obs)} 个体，分布在 {len(groups)} 个横断面；
  全部 static，高 {OBST_H}m（在 grid z_min 0.1 与 z_max 2.5 之间，激光稳打）。
  中心线段序：{os.linesep.join('      ' + f"{a:.1f}~{b:.1f}m {d}" for a, b, d in spans)}

  各断面可通行口宽（口宽低于 {SEAL:.1f}m 则膨胀后无 free 链，A* 无路）：
{os.linesep.join(lines) if lines else '      无'}

{os.linesep.join('  ' + t for t in notes)}
  出生点 x {sp[0]:.2f} y {sp[1]:.2f} yaw {sp[2]:.2f}，由中心线 s={sp[3]:.1f}m 取样得到，
  必然落在路上；第一段是直道，障碍从更远处开始。
-->
<sdf version="1.6">
  <world name="obstacle_{name}">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <include><uri>model://sun</uri></include>

{rg.ground_plane()}"""


def build_one(name, args):
    sections, ds, prof = road_of(PRESETS[name])
    preset = PRESETS[name]
    pairs, end = rg.centerline_segments(sections, ds)
    if not rg.validate(pairs, end, prof):
        print(f"[{name}] 道路几何校验未通过，仍写出文件供调试", file=sys.stderr)
    nodes, total = rg.polyline(pairs)
    obs = expand(preset['obstacles'], prof, nodes)
    groups = cross_sections(obs, prof, nodes)
    bad = constraint_check(obs, prof, nodes)
    sp = rg.sample(nodes, preset['spawn_s'])
    notes = OBSTACLE_NOTES.get(name, [])

    report(name, preset, sections, prof, total, obs, groups, sp)
    if bad:
        print("\n⚠ 弯道约束违规（本场景故意破例的，请补 ok_on_arc=True + 理由）：",
              file=sys.stderr)
        for b in bad:
            print(f"    · {b}", file=sys.stderr)
    if not args.no_ascii:
        rg.print_ascii(nodes, total, prof, obs, sp, name)
    if args.ascii:
        return
    out_world = args.out or os.path.join(WORLD_DIR, f"obstacle_{name}.world")
    os.makedirs(os.path.dirname(out_world), exist_ok=True)
    body = rg.road_body(pairs, nodes, prof) + obstacle_models(obs, nodes)
    hdr = world_header(name, sections, prof, pairs, obs, groups,
                       sp + (preset['spawn_s'],), total, notes)
    with open(out_world, "w") as f:
        f.write(hdr + "".join(body) + "  </world>\n</sdf>\n")
    print(f"wrote {os.path.relpath(out_world, os.getcwd())}  "
          f"（道路 {len(pairs) * 3} box + 障碍 {len(obs)} 个体）")
    rg.check_xml(out_world)
    if args.no_launch:
        return
    made = 0
    for mode in ('road', 'goal'):
        lp = os.path.join(LAUNCH_DIR, f"obstacle_{name}_{mode}.launch")
        if rg.build_launch(name, sp, out_world, lp, mode, notes):
            rg.check_xml(lp)
            made += 1
    print(f"[{name}] world 已写出；launch 新写/更新 {made} 个，"
          f"XML well-formed 检查通过")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="生成走廊内带静态障碍的闭合道路 world + launch")
    ap.add_argument("-p", "--preset", choices=sorted(PRESETS), default=None,
                    help="场景预设（默认 None = 生成全部）")
    ap.add_argument("-o", "--out", default=None, help="输出 .world 路径（仅单 preset）")
    ap.add_argument("--ascii", action="store_true", help="只打印俯视示意图，不写文件")
    ap.add_argument("--no-ascii", action="store_true", help="不打印俯视示意图")
    ap.add_argument("--no-launch", action="store_true", help="只生成 world，不出 launch")
    ap.add_argument("--force-launch", action="store_true",
                    help="连没有 AUTO-GENERATED 标记的 launch 也重写（同 gen_road_world）")
    args = ap.parse_args()
    rg.FORCE_LAUNCH = args.force_launch   # 标记检查在 gen_road_world 里，改那边
    names = [args.preset] if args.preset else sorted(PRESETS)
    if args.out and len(names) > 1:
        print("-o 仅单 preset 时可用", file=sys.stderr)
        sys.exit(2)
    for n in names:
        build_one(n, args)
