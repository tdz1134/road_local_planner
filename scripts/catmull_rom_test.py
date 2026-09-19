#!/usr/bin/env python3
"""
Catmull-Rom 样条插值可视化测试
1:1 复现 unk_nav/src/curve_fit.cpp fitSpline() 的算法逻辑。
对比不同控制点分布下的插值效果：均匀、急弯、间距悬殊、噪声、S 弯等。
另附一张方法对比图：CR / Akima / 自然三次样条(C2) / PCHIP / Clothoid 的曲线与曲率-弧长对比。
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from scipy.integrate import cumulative_trapezoid
from scipy.optimize import least_squares
from scipy.interpolate import CubicSpline, Akima1DInterpolator, PchipInterpolator


# ─── 核心算法：与 fitSpline 完全一致 ────────────────────────────────────────


def catmull_rom(control_pts: np.ndarray, start_tangent: float, spacing: float = 0.1):
    """
    Parameters
    ----------
    control_pts : (n, 2) ndarray — 跳点坐标 P[0]=车辆原点
    start_tangent : float — 起点切向角 (rad)，模拟车头朝向
    spacing : float — 密采样目标间距

    Returns
    -------
    dense : (m, 2) ndarray — 密采样后的曲线上点
    T : (n, 2) ndarray — 各控制点的切向量（用于可视化）
    """
    pts = np.asarray(control_pts, dtype=np.float64)
    n = len(pts)
    if n < 2:
        return pts, np.zeros_like(pts)

    sp = max(spacing, 1e-6)

    # 1) 计算切向量 T[i]
    T = np.zeros((n, 2))
    # 起点：方向 = start_tangent，模长 = 首段弦长
    s0 = np.linalg.norm(pts[1] - pts[0])
    T[0] = s0 * np.array([np.cos(start_tangent), np.sin(start_tangent)])
    # 中间点：中心差分 0.5*(P[i+1] - P[i-1])
    for i in range(1, n - 1):
        T[i] = 0.5 * (pts[i + 1] - pts[i - 1])
    # 末点：外推 P[n-1] - P[n-2]
    T[n - 1] = pts[n - 1] - pts[n - 2]

    # 2) 逐段三次 Hermite 密采样
    dense = [pts[0].copy()]
    for i in range(n - 1):
        P0, P1 = pts[i], pts[i + 1]
        M0, M1 = T[i], T[i + 1]
        seg_len = np.linalg.norm(P1 - P0)
        k = max(1, int(np.ceil(seg_len / sp)))
        for j in range(1, k + 1):
            t = j / k
            t2, t3 = t * t, t * t * t
            h00 = 2 * t3 - 3 * t2 + 1
            h10 = t3 - 2 * t2 + t
            h01 = -2 * t3 + 3 * t2
            h11 = t3 - t2
            point = h00 * P0 + h10 * M0 + h01 * P1 + h11 * M1
            dense.append(point)

    dense = np.array(dense)
    # 端点钉死
    dense[0] = pts[0]
    dense[-1] = pts[-1]
    return dense, T


def catmull_rom_chord(control_pts: np.ndarray, start_tangent: float, spacing: float = 0.1):
    """弦长参数化（非均匀）Catmull-Rom —— 1:1 复现当前 fitSpline。
    切向 G = 相邻两段单位割线平均（起点钉死车头、末点取末段单位方向）；
    每段端点导数再按本段弦长 D[i] 缩放（非均匀参数化，根治短末段曲率爆炸）。
    返回 dense 曲线点 与 G（切向量，用于画箭头）。"""
    pts = np.asarray(control_pts, dtype=np.float64)
    n = len(pts)
    if n < 2:
        return pts, np.zeros_like(pts)
    sp = max(spacing, 1e-6)

    # 各段弦长 D[i] 与单位割线 u[i]
    D = np.array([np.linalg.norm(pts[i + 1] - pts[i]) for i in range(n - 1)])
    u = np.array([(pts[i + 1] - pts[i]) / max(D[i], 1e-9) for i in range(n - 1)])

    # 切向量 G（弦长空间：中间点=相邻两段单位割线平均）
    G = np.zeros((n, 2))
    G[0] = np.array([np.cos(start_tangent), np.sin(start_tangent)])  # 单位模长
    for i in range(1, n - 1):
        G[i] = 0.5 * (u[i - 1] + u[i])
    G[n - 1] = u[n - 2]

    # 逐段三次 Hermite：端点导数 = G × 本段弦长 D[i]
    dense = [pts[0].copy()]
    for i in range(n - 1):
        P0, P1 = pts[i], pts[i + 1]
        M0, M1 = G[i] * D[i], G[i + 1] * D[i]
        k = max(1, int(np.ceil(D[i] / sp)))
        for j in range(1, k + 1):
            t = j / k
            t2, t3 = t * t, t * t * t
            h00 = 2 * t3 - 3 * t2 + 1
            h10 = t3 - 2 * t2 + t
            h01 = -2 * t3 + 3 * t2
            h11 = t3 - t2
            dense.append(h00 * P0 + h10 * M0 + h01 * P1 + h11 * M1)
    dense = np.array(dense)
    dense[0] = pts[0]
    dense[-1] = pts[-1]
    return dense, G


def catmull_rom_centripetal(control_pts: np.ndarray, start_tangent: float, spacing: float = 0.1):
    """向心参数化 (alpha=0.5) Catmull-Rom — 对比用，防止大间距比自交"""
    pts = np.asarray(control_pts, dtype=np.float64)
    n = len(pts)
    if n < 2:
        return pts
    sp = max(spacing, 1e-6)
    alpha = 0.5

    dense = [pts[0].copy()]
    for i in range(n - 1):
        # 需要 4 个点: P[i-1], P[i], P[i+1], P[i+2]
        # 端点补镜像虚拟点
        pm1 = pts[i - 1] if i > 0 else 2 * pts[0] - pts[1]
        p0 = pts[i]
        p1 = pts[i + 1]
        p2 = pts[i + 2] if i + 2 < n else 2 * pts[-1] - pts[-2]

        # 向心 knot spacing
        def knot_dist(a, b):
            return max(np.linalg.norm(b - a) ** alpha, 1e-6)

        t_prev = 0.0
        t_cur = t_prev + knot_dist(pm1, p0)
        t_next = t_cur + knot_dist(p0, p1)
        t_last = t_next + knot_dist(p1, p2)

        seg_len = np.linalg.norm(p1 - p0)
        k = max(1, int(np.ceil(seg_len / sp)))
        for j in range(1, k + 1):
            t = t_cur + (j / k) * (t_next - t_cur)
            # 向心 Hermite via A2(t) formulation (Barry-Goldman de Casteljau)
            A1 = (t_cur - t) / (t_cur - t_prev) * pm1 + (t - t_prev) / (t_cur - t_prev) * p0
            A2 = (t_next - t) / (t_next - t_cur) * p0 + (t - t_cur) / (t_next - t_cur) * p1
            A3 = (t_last - t) / (t_last - t_next) * p1 + (t - t_next) / (t_last - t_next) * p2
            B1 = (t_next - t) / (t_next - t_cur) * A1 + (t - t_cur) / (t_next - t_cur) * A2
            B2 = (t_last - t) / (t_last - t_next) * A2 + (t - t_next) / (t_last - t_next) * A3
            C = (t_next - t) / (t_next - t_cur) * B1 + (t - t_cur) / (t_next - t_cur) * B2
            dense.append(C)

    return np.array(dense)


# ─── 可视化辅助 ─────────────────────────────────────────────────────────────


def plot_tangents(ax, pts, T, scale=1.0, color='green', label='tangents'):
    """画切向量箭头（每个控制点都画，箭头置顶，并标序号以确认无遗漏）"""
    for i in range(len(pts)):
        dx, dy = T[i] * scale
        # 箭头本体：zorder=10 保证压在曲线/图例之上不被遮挡
        ax.annotate('', xy=(pts[i, 0] + dx, pts[i, 1] + dy), xytext=tuple(pts[i]),
                    arrowprops=dict(arrowstyle='->', color=color, lw=1.8,
                                    zorder=10, shrinkA=0, shrinkB=0),
                    zorder=10, label=label if i == 0 else '')
        # 箭头末端小圆点，避免与曲线混淆
        ax.plot(pts[i, 0] + dx, pts[i, 1] + dy, 'o', color=color, markersize=4, zorder=10)
        # 点序号标签：确认每个控制点都有对应切向量
        ax.annotate(str(i), xy=tuple(pts[i]), xytext=(pts[i, 0] - 0.12, pts[i, 1] - 0.22),
                    fontsize=8, color='red', zorder=11)
    # legend 只出现一次
    ax.plot([], [], 'g->', label=label)


def plot_case(ax, title, pts, start_tangent, spacing=0.15, show_centripetal=False):
    """在给定 ax 上画一个测试用例"""
    pts = np.array(pts)
    dense, T = catmull_rom(pts, start_tangent, spacing)

    # 折线（原始控制点连线）做参考
    ax.plot(pts[:, 0], pts[:, 1], 'k--o', markersize=6, linewidth=1, alpha=0.4, zorder=1, label='polyline')
    # Catmull-Rom (uniform)
    ax.plot(dense[:, 0], dense[:, 1], 'b-', linewidth=2, zorder=2, label='CR (uniform)')
    # 向心版对比
    if show_centripetal:
        dense_c = catmull_rom_centripetal(pts, start_tangent, spacing)
        ax.plot(dense_c[:, 0], dense_c[:, 1], 'r-', linewidth=2, alpha=0.7, zorder=2, label='CR (centripetal)')
    # 切向量
    plot_tangents(ax, pts, T, scale=0.6, color='green')

    ax.set_title(title, fontsize=11)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=8, framealpha=0.9)


# ─── 方法对比：CR / Akima / 自然三次样条 / PCHIP / Clothoid ───────────────────


def _chord_param(pts):
    """弦长累加参数化 t_i。"""
    d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(d)])


def resample_by_arc(dense, n=300):
    """按弧长均匀重采样，使曲率-弧长曲线可比。"""
    dense = np.asarray(dense, float)
    seg = np.linalg.norm(np.diff(dense, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] < 1e-9:
        return dense[:1], s[:1]
    s_new = np.linspace(0.0, s[-1], n)
    x = np.interp(s_new, s, dense[:, 0])
    y = np.interp(s_new, s, dense[:, 1])
    return np.column_stack([x, y]), s_new


def signed_curvature(dense):
    """带符号曲率 κ(s)（中心差分）。返回 (弧长, 曲率)。"""
    xy, s = resample_by_arc(dense)
    x, y = xy[:, 0], xy[:, 1]
    dx, dy = np.gradient(x, s), np.gradient(y, s)
    ddx, ddy = np.gradient(dx, s), np.gradient(dy, s)
    k = (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** 1.5
    return s, k


def interp_cr(pts, start_tangent, spacing=0.05):
    dense, _ = catmull_rom(pts, start_tangent, spacing)
    return dense


def interp_scipy(pts, kind, start_tangent):
    """Akima / natural-cubic / pchip：对 x(t),y(t) 分别做参数化插值。"""
    t = _chord_param(pts)
    ts = np.linspace(t[0], t[-1], 300)
    if kind == 'akima':
        it = Akima1DInterpolator(t, pts)
    elif kind == 'cubic':
        it = CubicSpline(t, pts, bc_type='natural')
    elif kind == 'pchip':
        it = PchipInterpolator(t, pts)
    else:
        raise ValueError(kind)
    xy = it(ts)
    return np.column_stack([xy[:, 0], xy[:, 1]])


def _clothoid_integrate(s_knot, k_knot, s_fine, p0, theta0):
    """给定节点曲率 k_knot（κ 在弧长上线性⇒每段即一条缓和曲线），积分出 xy 与航向。"""
    kap = np.interp(s_fine, s_knot, k_knot)
    theta = theta0 + cumulative_trapezoid(kap, s_fine, initial=0.0)
    x = p0[0] + cumulative_trapezoid(np.cos(theta), s_fine, initial=0.0)
    y = p0[1] + cumulative_trapezoid(np.sin(theta), s_fine, initial=0.0)
    return np.column_stack([x, y]), theta


def interp_clothoid(pts, start_tangent, n_fine=600):
    """曲率沿弧长线性(C2 连续)的缓和曲线样条拟合跳点。
    未知量=各节点曲率 k_i；残差=积分得到的节点位置与实际跳点之差。
    注：弧长节点用弦长近似，故为近似过点（对可视化对比足够）。"""
    pts = np.asarray(pts, float)
    s = _chord_param(pts)
    sf = np.linspace(s[0], s[-1], n_fine)

    # 初值：自然三次样条在各节点的曲率
    cs = CubicSpline(s, pts, bc_type='natural')
    d1 = cs.derivative(1)(s)  # (n, 2)
    d2 = cs.derivative(2)(s)  # (n, 2)
    dx, dy = d1[:, 0], d1[:, 1]
    ddx, ddy = d2[:, 0], d2[:, 1]
    k0 = (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** 1.5

    def resid(kk):
        xy, _ = _clothoid_integrate(s, kk, s, pts[0], start_tangent)
        return (xy[1:] - pts[1:]).ravel()  # 首点由积分起点保证，残差从第 2 点起

    sol = least_squares(resid, k0, method='lm')
    xy, _ = _clothoid_integrate(s, sol.x, sf, pts[0], start_tangent)
    return xy


def build_comparison(pts, start_tangent, suptitle, save_path):
    """左右两栏：上=曲线叠加，下=曲率-弧长。"""
    pts = np.asarray(pts, float)
    methods = [
        ('CR (uniform)', interp_cr(pts, start_tangent), 'b-', 2.2),
        ('Akima', interp_scipy(pts, 'akima', start_tangent), 'g-', 1.8),
        ('Natural cubic (C2)', interp_scipy(pts, 'cubic', start_tangent), 'c-', 1.8),
        ('PCHIP', interp_scipy(pts, 'pchip', start_tangent), 'm-', 1.8),
        ('Clothoid (G2)', interp_clothoid(pts, start_tangent), 'r-', 2.0),
    ]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6.5))
    # 左：曲线
    ax1.plot(pts[:, 0], pts[:, 1], 'k--o', markersize=7, alpha=0.5, zorder=1, label='hops')
    for name, dense, style, lw in methods:
        ax1.plot(dense[:, 0], dense[:, 1], style, linewidth=lw, alpha=0.85, zorder=2, label=name)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=9)
    ax1.set_title('Curves')
    # 右：曲率 vs 弧长
    for name, dense, style, lw in methods:
        s, k = signed_curvature(dense)
        ax2.plot(s, k, style, linewidth=lw, alpha=0.85, label=name)
    # 标出跳点所在弧长（曲率跳变处 = C1 不连续的证据）
    s_hop = _chord_param(pts)
    for sh in s_hop[1:-1]:
        ax2.axvline(sh, color='gray', ls=':', linewidth=0.8, alpha=0.6)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlabel('arc length s (m)')
    ax2.set_ylabel('curvature kappa (1/m)')
    ax2.set_title('Curvature vs arc length (jumps at knots = only C1)')
    ax2.legend(fontsize=9)
    fig.suptitle(suptitle, fontsize=13)
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.savefig(save_path, dpi=150, bbox_inches='tight')


def build_param_comparison(pts, start_tangent, suptitle, save_path):
    """均匀参数化 CR vs 弦长参数化 CR：左=曲线叠加，右=曲率-弧长。
    只比两个参数化（切向规则同族），隔离出‘参数化’这一个变量。返回两版峰值|κ|。"""
    pts = np.asarray(pts, float)
    du, _ = catmull_rom(pts, start_tangent, 0.05)
    dc, _ = catmull_rom_chord(pts, start_tangent, 0.05)
    su, ku = signed_curvature(du)
    sc, kc = signed_curvature(dc)
    peak_u, peak_c = float(np.max(np.abs(ku))), float(np.max(np.abs(kc)))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6.5))
    ax1.plot(pts[:, 0], pts[:, 1], 'k--o', markersize=7, alpha=0.5, zorder=1, label='hops')
    ax1.plot(du[:, 0], du[:, 1], 'b-', linewidth=2.2, alpha=0.9, zorder=2,
             label=f'CR uniform (peak|k|={peak_u:.2f})')
    ax1.plot(dc[:, 0], dc[:, 1], 'r-', linewidth=2.2, alpha=0.9, zorder=3,
             label=f'CR chord-length (peak|k|={peak_c:.2f})')
    ax1.set_aspect('equal'); ax1.grid(True, alpha=0.3); ax1.legend(fontsize=9)
    ax1.set_title('Curves')

    ax2.plot(su, ku, 'b-', linewidth=2.2, alpha=0.9, label='CR uniform')
    ax2.plot(sc, kc, 'r-', linewidth=2.2, alpha=0.9, label='CR chord-length')
    for sh in _chord_param(pts)[1:-1]:
        ax2.axvline(sh, color='gray', ls=':', linewidth=0.8, alpha=0.6)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlabel('arc length s (m)')
    ax2.set_ylabel('curvature kappa (1/m)')
    ax2.set_title(f'Curvature vs arc length (uniform {peak_u:.2f} vs chord {peak_c:.2f})')
    ax2.legend(fontsize=9)
    fig.suptitle(suptitle, fontsize=13)
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    return peak_u, peak_c


# ─── 测试用例 ───────────────────────────────────────────────────────────────

cases = []

# Case 1: 均匀分布微弯（最 normal 的情况）
pts1 = np.array([[0, 0], [1, 0.2], [2, 0.5], [3, 1.0], [4, 1.3]])
cases.append(("Case 1: Uniform gentle curve", pts1, 0.0, 0.15, False))

# Case 2: 直角急弯 — 可能过冲鼓包
pts2 = np.array([[0, 0], [1.5, 0], [3, 0], [3, 1.5], [3, 3]])
cases.append(("Case 2: Sharp 90° corner (overshoot?)", pts2, 0.0, 0.15, False))

# Case 3: 间距悬殊 — 前密后疏，看自交风险
pts3 = np.array([[0, 0], [0.3, 0.1], [0.6, 0.3], [2.5, 2.0], [5.0, 2.5]])
cases.append(("Case 3: Uneven spacing (crunode risk)", pts3, 0.0, 0.15, True))

# Case 4: S 形曲线（CR 经典好用场景）
pts4 = np.array([[0, 0], [1, 1.5], [2, 1.5], [3, 0], [4, -1.5], [5, -1.5]])
cases.append(("Case 4: S-curve", pts4, np.pi / 4, 0.15, False))

# Case 5: 带噪声的点（模拟传感器抖动）
np.random.seed(42)
pts5_clean = np.array([[0, 0], [1, 0.5], [2, 1], [3, 0.5], [4, 0]])
pts5 = pts5_clean + np.random.randn(5, 2) * 0.3
cases.append(("Case 5: Noisy control points", pts5, 0.1, 0.15, False))

# Case 6: 两点退化（仅一段，就是普通三次 Hermite）
pts6 = np.array([[0, 0], [2, 1]])
cases.append(("Case 6: Only 2 points (= single Hermite)", pts6, 0.3, 0.15, False))

# Case 7: 回头弯 — 控制点几乎反向
pts7 = np.array([[0, 0], [2, 0], [3, 1], [2, 2], [0, 2], [-1, 1]])
cases.append(("Case 7: Hairpin turn (U-turn)", pts7, 0.0, 0.15, True))

# Case 8: 均匀密集点 — 模拟链式前瞻跳点
pts8 = np.column_stack([
    np.linspace(0, 5, 8),
    0.8 * np.sin(np.linspace(0, np.pi, 8))
])
cases.append(("Case 8: Dense hops (chain lookahead sim)", pts8, 0.0, 0.12, False))


# ─── 绘图 ────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    fig, axes = plt.subplots(2, 4, figsize=(18, 9))
    axes = axes.flatten()

    for idx, (title, pts, tangent, sp, show_c) in enumerate(cases):
        if idx >= len(axes):
            break
        plot_case(axes[idx], title, pts, tangent, sp, show_c)

    # 隐藏多余子图
    for j in range(len(cases), len(axes)):
        axes[j].set_visible(False)

    plt.suptitle("Catmull-Rom Spline (fitSpline algorithm) — Various Control Point Distributions",
                 fontsize=13, y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.savefig("/home/t/projects/road_local_planner/scripts/catmull_rom_test.png", dpi=150, bbox_inches='tight')
    plt.close(fig)

    # 方法对比图（急弯 + 间距悬殊 两个能拉开差距的场景）
    build_comparison(
        np.array([[0, 0], [1.5, 0], [3, 0], [3, 1.5], [3, 3]]), 0.0,
        "Method comparison — Sharp 90° corner",
        "/home/t/projects/road_local_planner/scripts/catmull_rom_compare_corner.png")
    plt.close('all')
    build_comparison(
        np.array([[0, 0], [0.3, 0.1], [0.6, 0.3], [2.5, 2.0], [5.0, 2.5]]), 0.0,
        "Method comparison — Uneven spacing",
        "/home/t/projects/road_local_planner/scripts/catmull_rom_compare_uneven.png")
    plt.close('all')
    # 前段等距、仅最后两点挨得近（对应链式前瞻末跳被障碍/终点截短）
    build_comparison(
        np.array([[0, 0], [1.2, 0.2], [2.4, 0.5], [3.6, 0.9], [3.8, 1.0]]), 0.0,
        "Method comparison — Equal spacing + short last segment",
        "/home/t/projects/road_local_planner/scripts/catmull_rom_compare_shortlast.png")
    plt.close('all')

    # 均匀参数化 CR vs 弦长参数化 CR（本次 curve_fit 改动的前后对比）
    base = "/home/t/projects/road_local_planner/scripts/"
    pu, pc = build_param_comparison(
        np.array([[0, 0], [1.2, 0.2], [2.4, 0.5], [3.6, 0.9], [3.8, 1.0]]), 0.0,
        "Parameterization — Equal spacing + short last segment",
        base + "cr_uniform_vs_chord_shortlast.png")
    print(f"  shortlast: uniform peak|k|={pu:.2f}  chord peak|k|={pc:.2f}")
    pu2, pc2 = build_param_comparison(
        np.array([[0, 0], [1, 0.2], [2, 0.5], [3, 1.0], [4, 1.3]]), 0.0,
        "Parameterization — Near-uniform hops (no regression)",
        base + "cr_uniform_vs_chord_even.png")
    print(f"  even:      uniform peak|k|={pu2:.2f}  chord peak|k|={pc2:.2f}")

    print("Saved: catmull_rom_test.png, catmull_rom_compare_{corner,uneven,shortlast}.png, "
          "cr_uniform_vs_chord_{shortlast,even}.png")
    plt.show()
