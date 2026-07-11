#!/usr/bin/env python3
"""ARES V3 装配预览渲染 + 打印分盘。

  python render_preview.py
生成：
  hardware/drawings/装配预览/assembly_*.png   整车装配多视角
  hardware/drawings/装配预览/part_*.png       每个打印件单件图（打印姿态）
  Print required/plates/PLATE_*.stl           ≤250x250 打印盘（平躺、少支撑）
"""
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import font_manager

import trimesh
from trimesh.transformations import rotation_matrix as rot

import build_parts as bp

HERE = Path(__file__).resolve().parent
DRAW = HERE.parents[0] / "drawings" / "装配预览"
PLATES = bp.OUT / "plates"


def setup_font():
    for name in ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                 "Source Han Sans SC", "WenQuanYi Zen Hei", "PingFang SC"]:
        if name in {f.name for f in font_manager.fontManager.ttflist}:
            plt.rcParams["font.family"] = name
            plt.rcParams["axes.unicode_minus"] = False
            return True
    return False


HAS_CJK = setup_font()
L = lambda zh, en: zh if HAS_CJK else en

LIGHT = np.array([0.4, -0.3, 0.85])
LIGHT = LIGHT / np.linalg.norm(LIGHT)


def add_meshes(ax, meshes):
    """把所有网格合成一个 collection，跨零件按面深度统一排序，内部器件才不会被墙"透视"。"""
    tris, cols = [], []
    for mesh, color, _ in meshes:
        t = mesh.vertices[mesh.faces]
        n = mesh.face_normals
        lam = 0.45 + 0.55 * np.clip(n @ LIGHT, 0, 1)
        base = np.array(color) / 255.0
        c = np.clip(lam[:, None] * base[None, :], 0, 1)
        tris.append(t)
        cols.append(np.hstack([c, np.ones((len(c), 1))]))
    pc = Poly3DCollection(np.vstack(tris), facecolors=np.vstack(cols), edgecolors="none")
    ax.add_collection3d(pc)


def render(meshes, out, elev=22, azim=-60, title="", ortho=False, ground=True):
    fig = plt.figure(figsize=(10, 8), dpi=130)
    ax = fig.add_subplot(111, projection="3d")
    allpts = np.vstack([m.vertices for m, _, _ in meshes])
    lo, hi = allpts.min(0), allpts.max(0)
    ctr, rng = (lo + hi) / 2, (hi - lo).max() * 0.62
    if ground:
        g = 0.0
        gx = [ctr[0] - rng, ctr[0] + rng]
        gy = [ctr[1] - rng, ctr[1] + rng]
        ax.plot_surface(np.array([[gx[0], gx[1]], [gx[0], gx[1]]]),
                        np.array([[gy[0], gy[0]], [gy[1], gy[1]]]),
                        np.full((2, 2), g), color="#e8e4da", alpha=0.6, zorder=-5)
    add_meshes(ax, meshes)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlim(ctr[0] - rng, ctr[0] + rng)
    ax.set_ylim(ctr[1] - rng, ctr[1] + rng)
    ax.set_zlim(max(ctr[2] - rng, -10), ctr[2] + rng)
    ax.view_init(elev=elev, azim=azim)
    if ortho:
        ax.set_proj_type("ortho")
    ax.set_axis_off()
    ax.set_title(title, fontsize=13, color="#0b0b0b")
    fig.tight_layout(pad=0.2)
    fig.savefig(out, facecolor="white")
    plt.close(fig)
    print("  ->", out.name)


def world_meshes(parts, acc):
    ms = []
    for p in parts:
        m = p["mesh"].copy()
        m.apply_transform(p["world"])
        ms.append((m, p["color"], 1.0))
    for a in acc:
        m = a["mesh"].copy()
        m.apply_transform(a["world"])
        ms.append((m, a["color"], 1.0))
    return ms


def main():
    DRAW.mkdir(parents=True, exist_ok=True)
    PLATES.mkdir(parents=True, exist_ok=True)
    parts = bp.build_all()
    bp.export(parts)                       # 同时导出单件 STL + 打印姿态
    acc = bp.accessories()

    # ---------- 整车装配视图 ----------
    ms = world_meshes(parts, acc)
    render(ms, DRAW / "assembly_1_前左等轴.png", 22, -55,
           L("ARES V3 整车装配 — 前左等轴视图", "ARES V3 assembly - iso front-left"))
    render(ms, DRAW / "assembly_2_后右等轴.png", 22, 130,
           L("整车装配 — 后右等轴（开关护架/充电口侧）", "iso rear-right"))
    render(ms, DRAW / "assembly_3_右侧视.png", 0, -90, ortho=True,
           title=L("右侧视 — Rocker-Bogie 几何（轮距 160，A/B 铰链）", "side view"))
    render(ms, DRAW / "assembly_4_正前视.png", 0, 0, ortho=True,
           title=L("正前视 — 三向超声波 / 轮距", "front view"))
    render(ms, DRAW / "assembly_5_俯视.png", 90, -90, ortho=True,
           title=L("俯视 — 平衡差动杆 / 桅杆 / 甲虫穹顶锚点", "top view"))
    # 舱内剖视：隐藏顶盖/桅杆/杆/相机架
    hide = {"02_顶盖_lid", "09_平衡差动杆_balance_bar", "10_桅杆_mast",
            "11_相机支架_cam_bracket"}
    ms_open = world_meshes([p for p in parts if p["name"] not in hide], acc)
    render(ms_open, DRAW / "assembly_6_开盖内舱.png", 40, -50,
           title=L("开盖视图 — B1 电池 / F1 驱动 / F2 主控 三层布局", "open cabin"))
    # 悬挂特写（只保留摆臂+轮+电机+主舱）
    keep = {"01_主舱_cabin", "05_右摇臂_rocker_R", "07_右转向架_bogie_R"}
    ms_sus = world_meshes([p for p in parts if p["name"] in keep],
                          [a for a in acc if a["name"] in ("wheel", "motor")])
    render(ms_sus, DRAW / "assembly_7_悬挂特写.png", 8, 118, ground=False,
           title=L("右侧悬挂特写 — A(M5)/B(M4) 铰链、电机站 M3 槽孔+Φ8 轴孔",
                   "suspension detail"))

    # ---------- 爆炸装配图（每个零件沿装配方向散开） ----------
    explode = {
        "01_主舱_cabin": (0, 0, 0), "02_顶盖_lid": (0, 0, 90),
        "03_楼层F1_deck": (0, 0, 35), "04_楼层F2_deck": (0, 0, 62),
        "05_右摇臂_rocker_R": (0, 60, 0), "06_左摇臂_rocker_L": (0, -60, 0),
        "07_右转向架_bogie_R": (0, 120, 0), "08_左转向架_bogie_L": (0, -120, 0),
        "09_平衡差动杆_balance_bar": (0, 0, 130), "10_桅杆_mast": (0, 0, 150),
        "11_相机支架_cam_bracket": (0, 0, 170),
        "12_开关护架_switch_guard": (-60, 0, 0),
        "13_超声波支架_sonar_bracket": (60, 0, 0),
    }
    ms_ex = []
    for p in parts:
        m = p["mesh"].copy()
        m.apply_transform(p["world"])
        m.apply_translation(explode.get(p["name"], (0, 0, 0)))
        ms_ex.append((m, p["color"], 1.0))
    render(ms_ex, DRAW / "assembly_0_爆炸图.png", 18, -55,
           title=L("爆炸装配图 — 13 个打印件的装配关系", "exploded view"), ground=False)

    # ---------- 单件图（打印姿态） ----------
    for p in parts:
        m = p["mesh_print"]
        e = m.bounding_box.extents
        ttl = (f"{p['name']}   {e[0]:.0f} x {e[1]:.0f} x {e[2]:.0f} mm"
               + (f"  (斜放 {p['print_rot_deg']:.0f}°)" if p.get("print_rot_deg") else "")
               + "\n" + p.get("note", ""))
        render([(m, p["color"], 1.0)], DRAW / f"part_{p['name']}.png",
               35, -60, title=ttl, ground=False)

    # ---------- 打印分盘（≤240x240，全部平躺） ----------
    layout = {
        "PLATE_1_主舱": ["01_主舱_cabin"],
        "PLATE_2_顶盖+小件": ["02_顶盖_lid", "11_相机支架_cam_bracket",
                            "12_开关护架_switch_guard"],
        "PLATE_3_楼层F1F2": ["03_楼层F1_deck", "04_楼层F2_deck"],
        "PLATE_4_右摇臂+差动杆": ["05_右摇臂_rocker_R", ("09_平衡差动杆_balance_bar", 90)],
        "PLATE_5_左摇臂+桅杆": ["06_左摇臂_rocker_L", "10_桅杆_mast"],
        "PLATE_6_转向架+超声波架": ["07_右转向架_bogie_R", "08_左转向架_bogie_L",
                                 "13_超声波支架_sonar_bracket"],
    }
    bynames = {p["name"]: p for p in parts}
    gap = 8.0
    print("\n打印盘：")
    for pname, items in layout.items():
        placed, ycur = [], 0.0
        for it in items:
            it, spin = it if isinstance(it, tuple) else (it, 0)
            m = bynames[it]["mesh_print"].copy()
            if spin:
                m.apply_transform(rot(math.radians(spin), (0, 0, 1)))
            e = m.bounding_box.extents
            m.apply_translation((-m.bounds[0][0], -m.bounds[0][1] + ycur, -m.bounds[0][2]))
            placed.append((m, bynames[it]["color"]))
            ycur += e[1] + gap
        plate = trimesh.util.concatenate([m for m, _ in placed])
        ext = plate.bounding_box.extents
        ok = ext[0] <= 246 and ext[1] <= 246
        plate.export(PLATES / f"{pname}.stl")
        print(f"  {pname:26s} {ext[0]:6.1f} x {ext[1]:6.1f} x {ext[2]:5.1f}  "
              f"{'✓' if ok else '✗✗ 超床!'}")
        render([(m, c, 1.0) for m, c in placed], DRAW / f"plate_{pname}.png",
               55, -75, title=f"{pname}  {ext[0]:.0f} x {ext[1]:.0f} mm", ground=False)


if __name__ == "__main__":
    main()
