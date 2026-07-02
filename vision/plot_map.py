#!/usr/bin/env python3
"""ARES 地图绘制：用遥测日志做航迹推算，画出行驶轨迹 + 超声波障碍点（"扫地机地图"）。

用法:
    python plot_map.py logs/telemetry_20260702_120000.jsonl -o map.png
    python plot_map.py logs/*.jsonl --speed 15

原理：没有编码器，速度用 油门(thr) × 标定速度(--speed，来自 T2 测试的
CM_PER_SEC_CRUISE) 估计；方向用 MPU6050 的 yaw；前向超声波读数投影成障碍点。
是演示级地图，误差随里程累积，适合几米范围的房间演示。
依赖: pip install matplotlib numpy
"""
import argparse
import glob
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager


def setup_cjk_font():
    """选一个可用的中文字体；找不到就回退英文标签（matplotlib 默认字体不含中文）。"""
    candidates = ["Microsoft YaHei", "SimHei", "PingFang SC", "Hiragino Sans GB",
                  "Noto Sans CJK SC", "Source Han Sans SC", "WenQuanYi Zen Hei"]
    installed = {f.name for f in font_manager.fontManager.ttflist}
    for name in candidates:
        if name in installed:
            plt.rcParams["font.family"] = name
            plt.rcParams["axes.unicode_minus"] = False
            return True
    return False


HAS_CJK = setup_cjk_font()
L = (lambda zh, en: zh if HAS_CJK else en)

# dataviz 参考色板（亮色面）
SURFACE = "#fcfcfb"
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#898781"
GRID, BASELINE = "#e1e0d9", "#c3c2b7"
C_PATH = "#2a78d6"       # 轨迹（系列1 蓝）
C_OBST = "#1baf7a"       # 障碍点（系列2 青）——浅底对比不足，故加深色描边+图例+计数标注
C_OBST_EDGE = "#0e6e4e"


def load_rows(patterns):
    rows = []
    for pat in patterns:
        for f in sorted(glob.glob(pat)):
            with open(f, encoding="utf-8") as fh:
                for line in fh:
                    try:
                        m = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if m.get("type") == "telemetry":
                        rows.append(m)
    return rows


def dead_reckon(rows, speed_cm_s, obstacle_max_cm):
    xs, ys = [0.0], [0.0]
    obst = set()
    x = y = 0.0
    last_t = None
    for m in rows:
        t = m.get("pc_ts")
        dt = 0.2 if (last_t is None or t is None) else min(max(t - last_t, 0.0), 1.0)
        last_t = t
        yaw = math.radians(float(m.get("yaw", 0.0)))
        v = float(m.get("thr", 0.0)) * speed_cm_s
        # 车头初始朝 +Y（图上朝上）；yaw 增大 = 逆时针
        x += -v * dt * math.sin(yaw)
        y += v * dt * math.cos(yaw)
        xs.append(x)
        ys.append(y)
        f = m.get("front_cm", -1)
        if isinstance(f, (int, float)) and 0 < f < obstacle_max_cm:
            ox = x - f * math.sin(yaw)
            oy = y + f * math.cos(yaw)
            obst.add((round(ox / 5) * 5, round(oy / 5) * 5))  # 5cm 网格去重
    return np.array(xs), np.array(ys), np.array(sorted(obst)) if obst else np.empty((0, 2))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("logs", nargs="+", help="telemetry JSONL 文件（可通配符）")
    ap.add_argument("-o", "--out", default="map.png")
    ap.add_argument("--speed", type=float, default=15.0,
                    help="满油门速度 cm/s（T2 实测后回填，与 config.h 的 CM_PER_SEC_CRUISE 一致）")
    ap.add_argument("--obstacle-max", type=float, default=150.0,
                    help="超过此距离(cm)的超声波读数不当作障碍")
    args = ap.parse_args()

    rows = load_rows(args.logs)
    if len(rows) < 5:
        sys.exit(f"遥测太少（{len(rows)} 条），先用 telemetry_logger.py 录制")
    xs, ys, obst = dead_reckon(rows, args.speed, args.obstacle_max)
    dist_m = float(np.sum(np.hypot(np.diff(xs), np.diff(ys)))) / 100.0

    fig, ax = plt.subplots(figsize=(8, 8), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)

    ax.plot(xs, ys, color=C_PATH, lw=2, label=L("行驶轨迹", "Path"), zorder=3)
    if len(obst):
        ax.scatter(obst[:, 0], obst[:, 1], s=64, marker="s", color=C_OBST,
                   edgecolors=C_OBST_EDGE, linewidths=1.2,
                   label=L(f"障碍点 ×{len(obst)}", f"Obstacles x{len(obst)}"), zorder=2)
    # 起终点：白圈隔离 + 直接标注
    ax.scatter([xs[0]], [ys[0]], s=90, color=C_PATH, edgecolors=SURFACE, linewidths=2, zorder=4)
    ax.scatter([xs[-1]], [ys[-1]], s=90, marker="D", color=C_PATH,
               edgecolors=SURFACE, linewidths=2, zorder=4)
    ax.annotate(L("起点", "Start"), (xs[0], ys[0]), textcoords="offset points",
                xytext=(8, 8), color=INK, fontsize=10)
    ax.annotate(L("终点", "End"), (xs[-1], ys[-1]), textcoords="offset points",
                xytext=(8, -16), color=INK, fontsize=10)   # 与起点错开，闭环轨迹时不重叠

    ax.set_title(L(f"ARES 区域地图 — 航迹推算（里程约 {dist_m:.1f} m）",
                   f"ARES Area Map - Dead Reckoning (approx. {dist_m:.1f} m)"),
                 color=INK, fontsize=13, pad=12)
    ax.set_xlabel("X (cm)", color=INK2)
    ax.set_ylabel("Y (cm)", color=INK2)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(True, color=GRID, lw=0.8)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(BASELINE)
    ax.set_aspect("equal", adjustable="datalim")
    leg = ax.legend(loc="upper right", frameon=False, fontsize=10)
    for t in leg.get_texts():
        t.set_color(INK2)

    fig.tight_layout()
    fig.savefig(args.out, facecolor=SURFACE)
    print(f"地图已保存 -> {args.out}   遥测 {len(rows)} 条 | 里程 {dist_m:.1f} m | 障碍点 {len(obst)}")


if __name__ == "__main__":
    main()
