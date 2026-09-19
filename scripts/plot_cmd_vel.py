#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""控制量记录与绘图：订阅 /cmd_vel + /unk_nav/state 若干秒 → CSV + PNG。

用途：诊断"转一下直行一下"等丝滑问题——看角速度是否锯齿/换向、
     状态串里 curve↔line(回退) 是否交替、限速来源 limit_by 切换。

用法（启动场景、车动起来之后，新终端里跑；项目根目录下）：
  source ~/projects/road_local_planner/devel/setup.bash
  python3 scripts/plot_cmd_vel.py 120

输出（默认落工作区根目录，带时间戳，重跑不覆盖）：
  cmdvel_<时间戳>.csv   逐帧原始数据 t,v,w,state
  cmdvel_<时间戳>.png   v/w 双曲线图（灰带=样条回退直线，橙线=limit_by 切换）
  cmdvel_<时间戳>.txt   诊断摘要（换向/间隔/回退占比/limit_by 统计）
分析时把三个文件（至少 txt+png）发出去即可。可选第二参指定前缀：
  python3 scripts/plot_cmd_vel.py 120 /tmp/run1   # → /tmp/run1.{csv,png,txt}
"""
import datetime
import os
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
# 默认前缀：当前目录下带时间戳，重复跑互不覆盖
PREFIX = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
    os.getcwd(), "cmdvel_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))

rows = []            # [t, v, w, state]
state_buf = [""]     # 最近一帧 /unk_nav/state 字符串
t0 = [None]


def cb_state(msg):
    state_buf[0] = msg.data


def cb_cmd(msg):
    now = time.time()          # 用墙钟：不受 Gazebo 仿真时钟跳变/暂停影响
    if t0[0] is None:
        t0[0] = now
    rows.append([now - t0[0], msg.linear.x, msg.angular.z, state_buf[0]])


def main():
    global rows  # 下方要对模块级 rows 重新绑定快照，不声明会成局部变量 UnboundLocalError
    rospy.init_node("plot_cmd_vel", anonymous=True)
    sub_cmd = rospy.Subscriber("/cmd_vel", Twist, cb_cmd, queue_size=100)
    sub_state = rospy.Subscriber("/unk_nav/state", String, cb_state, queue_size=10)
    rate = rospy.Rate(10)
    t_end = time.time() + DUR
    while not rospy.is_shutdown() and time.time() < t_end:
        rate.sleep()

    # 回调线程窗口结束后仍会往全局 rows 追加：先注销订阅再拍快照，
    # 否则后面画图循环的 len(rows) 会超过 ts 快照长度导致 IndexError。
    sub_cmd.unregister()
    sub_state.unregister()
    rows = [r for r in rows]

    if not rows:
        print("!! 窗口内没收到任何 /cmd_vel，检查 nav_node 是否在跑")
        return

    # ── CSV ─
    csv_path = PREFIX + ".csv"
    with open(csv_path, "w") as f:
        f.write("t,v,w,state\n")
        for r in rows:
            f.write("%.3f,%.4f,%.4f,%s\n" % (r[0], r[1], r[2], r[3]))
    print("CSV  → %s  (%d 帧, %.0fs)" % (csv_path, len(rows), rows[-1][0]))

    # ── 数值诊断（终端打印 + 同步写 <prefix>.txt，发结果时信息齐全）──
    ts = [r[0] for r in rows]
    ws = [r[2] for r in rows]
    vs = [r[1] for r in rows]
    S = ["抓取时刻 %s，时长 %.0fs，%d 帧（≈%.0f Hz）"
         % (time.strftime("%Y-%m-%d %H:%M:%S"), ts[-1], len(rows), len(rows) / max(ts[-1], 1e-6))]
    # 角速度换向点（|w|>0.05 才算"在转"，避免零点噪声）
    flips, prev_sign, flip_detail = 0, 0, []
    for t, v, w, s in rows:
        sg = 1 if w > 0.05 else (-1 if w < -0.05 else 0)
        if sg != 0 and prev_sign != 0 and sg != prev_sign:
            flips += 1
            flip_detail.append((t, v, w))
        if sg != 0:
            prev_sign = sg
    gaps = [flip_detail[i + 1][0] - flip_detail[i][0] for i in range(len(flip_detail) - 1)]
    gaps_sorted = sorted(gaps)
    med = gaps_sorted[len(gaps_sorted) // 2] if gaps_sorted else 0.0
    # 角速度相邻帧跳变率
    dw_rate = []
    for i in range(len(ws) - 1):
        dt = ts[i + 1] - ts[i]
        if dt > 1e-6:
            dw_rate.append(abs(ws[i + 1] - ws[i]) / dt)
    # limit_by 序列：切换次数与分布（逐帧跳变 = 路径曲率估计在抖）
    lbs = []
    for r in rows:
        tok = [x for x in r[3].split() if x.startswith("limit_by=")]
        lbs.append(tok[0][9:] if tok else "")
    lb_sw = sum(1 for i in range(1, len(lbs)) if lbs[i] and lbs[i - 1] and lbs[i] != lbs[i - 1])
    lb_cnt = {}
    for x in lbs:
        if x:
            lb_cnt[x] = lb_cnt.get(x, 0) + 1
    noline = sum(1 for r in rows if r[3] and "curve" not in r[3])

    S.append("角速度换向: %d 次（%.1f 次/分）  间隔 中位%.1fs min%.2fs <1s占%d/%d"
             % (flips, flips / max(ts[-1] / 60.0, 1e-6), med,
                min(gaps) if gaps else 0.0,
                sum(1 for g in gaps if g < 1.0), len(gaps)))
    S.append("|dw/dt|: max %.2f  mean %.2f rad/s^2（w斜率限幅见 cmd_w_dot_max）"
             % (max(dw_rate) if dw_rate else 0.0,
                sum(dw_rate) / len(dw_rate) if dw_rate else 0.0))
    S.append("非curve帧（样条回退直线/IDLE）: %d / %d (%.1f%%)" % (noline, len(rows), 100.0 * noline / len(rows)))
    S.append("limit_by 切换: %d 次  分布: %s" % (lb_sw, lb_cnt))
    S.append("v 范围 [%.2f, %.2f] m/s   w 范围 [%.2f, %.2f] rad/s"
             % (min(vs), max(vs), min(ws), max(ws)))
    S.append("前 20 个换向点 (t, v, w):")
    for f in flip_detail[:20]:
        S.append("  t=%6.1f v=%.2f w=%+.2f" % f)
    for line in S:
        print(line)
    with open(PREFIX + ".txt", "w") as f:
        f.write("\n".join(S) + "\n")
    print("TXT  → %s.txt" % PREFIX)

    # ── 绘图 ──
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("!! 无 matplotlib，只出 CSV")
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(15, 7))
    ax1.plot(ts, vs, "-b", lw=1.2)
    ax1.set_ylabel("v [m/s]")
    ax1.grid(alpha=0.3)
    ax2.plot(ts, ws, "-r", lw=1.2)
    ax2.set_ylabel("w [rad/s]")
    ax2.set_xlabel("t [s]")
    ax2.grid(alpha=0.3)

    # 灰色竖带：连续"非 curve"区间（回退直线/IDLE）
    start = None
    for i, r in enumerate(rows):
        bad = bool(r[3]) and "curve" not in r[3]
        if bad and start is None:
            start = ts[i]
        if (not bad or i == len(rows) - 1) and start is not None:
            ax1.axvspan(start, ts[i], color="gray", alpha=0.35)
            ax2.axvspan(start, ts[i], color="gray", alpha=0.35)
            start = None
    # 黄竖线：limit_by 变化点
    prev_lb = ""
    for i, r in enumerate(rows):
        lb = [tok for tok in r[3].split() if tok.startswith("limit_by=")]
        lb = lb[0] if lb else ""
        if lb and prev_lb and lb != prev_lb:
            ax1.axvline(ts[i], color="orange", lw=0.8, alpha=0.7)
            ax2.axvline(ts[i], color="orange", lw=0.8, alpha=0.7)
        if lb:
            prev_lb = lb

    ax1.set_title("/cmd_vel %.0fs  (gray=no-curve fallback, orange=limit_by change)" % DUR)
    png_path = PREFIX + ".png"
    fig.savefig(png_path, dpi=110)
    print("PNG  → %s" % png_path)


if __name__ == "__main__":
    main()
