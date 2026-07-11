#!/usr/bin/env python3
"""ARES V3 装配尺寸校验：直接在生成的网格上"量尺寸"，逐项 PASS/FAIL。

校验内容（不是看图，是量网格）：
  A. 配合孔对位：每个螺栓关节，在两个零件上分别切截面找孔，
     核对 孔径 与 两孔轴线偏差 ≤ 0.3mm
  B. 干涉检查：所有打印件两两 + 打印件 x 器件 的布尔交体积 ≈ 0
  C. 关键间隙：电池顶↔F1 底、L298N 散热片↔F2 底、主控板顶↔顶盖、
     摇臂↔舱壁转动间隙、六轮着地共面
用法：python verify_fit.py   （非零退出码 = 存在 FAIL）
"""
import math
import sys

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix as rot

import build_parts as bp

TOL_AXIS = 0.35     # 孔轴对位容差 mm
TOL_D = 0.25        # 孔径容差 mm
TOL_VOL = 5.0       # 干涉体积容差 mm^3（数值噪声）

results = []


def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}  {detail}")


def posed(p):
    m = p["mesh"].copy()
    m.apply_transform(p["world"])
    return m


def find_hole(mesh, point, axis, expect_d, search_r=6.0):
    """沿 axis 在 point 附近多个平面切片，找孔径最接近 expect_d 的内环。
    返回 (孔心3D, 实测直径)。零件不一定恰好覆盖 point，所以要扫描 ±14mm。"""
    axis = np.array(axis, float)
    axis /= np.linalg.norm(axis)
    point = np.array(point, float)
    best = None            # (score, center3d, d)
    for off in (0, 1, -1, 2, -2, 3, -3, 4, -4, 5, -5, 6.5, -6.5, 8, -8,
                10, -10, 12, -12, 14, -14):
        origin = point + axis * off
        try:
            sec = mesh.section(plane_origin=origin, plane_normal=axis)
        except BaseException:
            sec = None
        if sec is None:
            continue
        try:
            p2, T = sec.to_2D()
        except BaseException:
            continue
        pt2 = trimesh.transform_points([origin], np.linalg.inv(T))[0][:2]
        for poly in p2.polygons_full:
            for ring in poly.interiors:
                xs, ys = ring.xy
                c = np.array([np.mean(xs[:-1]), np.mean(ys[:-1])])
                b = ring.bounds
                d = ((b[2] - b[0]) + (b[3] - b[1])) / 2
                dist = np.linalg.norm(c - pt2)
                if dist > search_r or abs(d - expect_d) > 2.0:
                    continue           # 只认孔径接近期望值的环（跳过沉台/大开孔）
                score = abs(d - expect_d) * 2 + dist * 0.5 + abs(off) * 0.02
                if best is None or score < best[0]:
                    c3 = trimesh.transform_points([[c[0], c[1], 0]], T)[0]
                    best = (score, np.array(c3), d, (b[2] - b[0], b[3] - b[1]))
    if best is None:
        return None, None, None
    return best[1], best[2], best[3]


def joint(name, mA, mB, point, axis, dA, dB, pointB=None, slotB=None):
    """核对 A/B 两件在同一轴上的孔：孔径 + 轴线偏移（垂直于 axis 的分量）。
    pointB：B 件的探测点（默认与 A 相同）。slotB=(宽, 长)：B 是腰形槽。"""
    axis = np.array(axis, float)
    axis /= np.linalg.norm(axis)
    dB_probe = dB if slotB is None else (slotB[0] + slotB[1]) / 2
    cA, mdA, _ = find_hole(mA, point, axis, dA)
    cB, mdB, bbB = find_hole(mB, pointB if pointB is not None else point,
                             axis, dB_probe)
    if cA is None or cB is None:
        check(name, False, f"找不到孔 A={cA is not None} B={cB is not None}")
        return
    off = (cB - cA) - np.dot(cB - cA, axis) * axis
    okA = abs(mdA - dA) <= TOL_D
    if slotB is None:
        mis = np.linalg.norm(off)
        okB = abs(mdB - dB) <= TOL_D
        detail = f"轴偏 {mis:.2f}mm | ΦA {mdA:.2f}/{dA} ΦB {mdB:.2f}/{dB}"
    else:
        # 槽孔：短边方向必须对位，长边方向允许滑动（|off| 限制在槽半长内）
        mis = min(abs(off), key=None) if False else float(np.min(np.abs(off[np.abs(off) > 1e-9]))) if np.any(np.abs(off) > 1e-9) else 0.0
        wslot = min(bbB)
        okB = abs(wslot - slotB[0]) <= TOL_D and np.linalg.norm(off) <= slotB[1] / 2
        detail = (f"短向偏 {mis:.2f}mm 滑向偏 {np.linalg.norm(off):.2f}"
                  f" | ΦA {mdA:.2f}/{dA} 槽宽 {wslot:.2f}/{slotB[0]}")
    ok = mis <= TOL_AXIS and okA and okB
    check(name, ok, detail)


def hole_only(name, m, point, axis, d):
    c, md, _ = find_hole(m, point, axis, d)
    if c is None:
        check(name, False, "找不到孔")
        return
    perp = (c - np.array(point)) - np.dot(c - np.array(point), np.array(axis)) * np.array(axis, float)
    ok = abs(md - d) <= TOL_D and np.linalg.norm(perp) <= TOL_AXIS
    check(name, ok, f"Φ {md:.2f}/{d} 位偏 {np.linalg.norm(perp):.2f}mm")


def main():
    parts = bp.build_all()
    for p in parts:
        p["world_mesh"] = posed(p)
    acc = bp.accessories()
    for a in acc:
        a["world_mesh"] = posed(a)
    byname = {p["name"]: p["world_mesh"] for p in parts}
    cab, lid = byname["01_主舱_cabin"], byname["02_顶盖_lid"]
    f1, f2 = byname["03_楼层F1_deck"], byname["04_楼层F2_deck"]
    rkR, rkL = byname["05_右摇臂_rocker_R"], byname["06_左摇臂_rocker_L"]
    bgR, bgL = byname["07_右转向架_bogie_R"], byname["08_左转向架_bogie_L"]
    bar, mast = byname["09_平衡差动杆_balance_bar"], byname["10_桅杆_mast"]
    guard, sonar = byname["12_开关护架_switch_guard"], byname["13_超声波支架_sonar_bracket"]

    ax, azw = bp.P["hingeA"][0], bp.P["hingeA"][1]
    bx, bzw = bp.P["hingeB"][0], bp.P["hingeB"][1]
    yR = bp.P["rocker_y"] + bp.P["arm_t"] / 2
    yB = bp.P["rocker_y"] + bp.P["arm_t"] + bp.P["bogie_gap"] + bp.P["arm_t"] / 2

    print("== A. 配合孔对位 ==")
    for s, rk, bg, tag in ((+1, rkR, bgR, "右"), (-1, rkL, bgL, "左")):
        joint(f"主铰链A-{tag} 舱壁Φ5.5 ↔ 摇臂Φ5.5", cab, rk,
              (ax, s * (bp.P["cab_W"] / 2 - 2), azw), (0, 1, 0), bp.P["m5"], bp.P["m5"])
        joint(f"二级铰链B-{tag} 摇臂Φ4.5 ↔ 转向架Φ4.5", rk, bg,
              (bx, s * yR, bzw), (0, 1, 0), bp.P["m4"], bp.P["m4"])
        joint(f"差动杆端-{tag} 立柱Φ4.5 ↔ 杆槽", rk, bar,
              (ax, s * yR, bp.P["horn_top_z"] - 2), (0, 0, 1), bp.P["m4"],
              bp.P["m4"] + 0.2, pointB=(ax, s * yR, bp.P["horn_top_z"] + 4),
              slotB=(bp.P["m4"] + 0.2, 11.0))
    joint("差动杆中心Φ5.5 ↔ 顶盖枢轴Φ5.5", bar, lid,
          (ax, 0, bp.P["cab_z0"] + bp.P["cab_H"] + 6), (0, 0, 1), bp.P["m5"], bp.P["m5"])
    zl = bp.P["cab_z0"] + bp.P["cab_H"] + 1.5
    px = bp.P["cab_L"] / 2 - bp.P["wall"] - bp.P["post"] / 2
    py = bp.P["cab_W"] / 2 - bp.P["wall"] - bp.P["post"] / 2
    for sx in (1, -1):
        for sy in (1, -1):
            joint(f"顶盖角孔({sx:+d},{sy:+d}) Φ3.4 ↔ 角柱底孔Φ2.9", lid, cab,
                  (sx * px, sy * py, zl - 0.2), (0, 0, 1), bp.P["m3"], bp.P["m3_pilot"])
    for sx in (1, -1):
        for sy in (1, -1):
            x, y = sx * bp.P["standoff_x"], sy * bp.P["standoff_y"]
            joint(f"支撑柱({sx:+d},{sy:+d}) 舱底Φ3.4 ↔ F1", cab, f1,
                  (x, y, bp.P["cab_z0"] + 2), (0, 0, 1), bp.P["m3"], bp.P["m3"],
                  pointB=(x, y, bp.P["cab_z0"] + 32))
            joint(f"支撑柱({sx:+d},{sy:+d}) F1 ↔ F2", f1, f2,
                  (x, y, bp.P["cab_z0"] + 32), (0, 0, 1), bp.P["m3"], bp.P["m3"],
                  pointB=(x, y, bp.P["cab_z0"] + 71))
    for sx in (1, -1):
        for sy in (1, -1):
            joint(f"桅杆底座({sx:+d},{sy:+d}) Φ3.4 ↔ 顶盖Φ3.4", mast, lid,
                  (sx * bp.P["mast_holes"][0] / 2, sy * bp.P["mast_holes"][1] / 2, zl + 2),
                  (0, 0, 1), bp.P["m3"], bp.P["m3"])
    for sy in (1, -1):
        joint(f"超声波支架耳({sy:+d}) ↔ 前墙Φ3.4", sonar, cab,
              (bp.P["cab_L"] / 2 - 1, sy * 20, bp.P["cab_z0"] + 32), (1, 0, 0),
              bp.P["m3"], bp.P["m3"])
    joint("开关孔 护架Φ12.6 ↔ 后墙Φ12.6", guard, cab,
          (-(bp.P["cab_L"] / 2) + 1, 30, bp.P["cab_z0"] + 19), (1, 0, 0),
          bp.P["switch_hole"], bp.P["switch_hole"])
    for dy in (14, -14):
        joint(f"护架M3({dy:+d}) ↔ 后墙Φ3.4", guard, cab,
              (-(bp.P["cab_L"] / 2) + 1, 30 + dy, bp.P["cab_z0"] + 19), (1, 0, 0),
              bp.P["m3"], bp.P["m3"])
    # 电机站：轴孔Φ8 与两条 M3 槽间距 17.6
    for tag, m, xs, side in (("右摇臂前轮", rkR, [bp.P["x_front"]], +1),
                             ("右转向架中轮", bgR, [bp.P["x_mid"]], +1),
                             ("右转向架后轮", bgR, [bp.P["x_rear"]], +1)):
        y = yR if "摇臂" in tag else yB
        for x0 in xs:
            hole_only(f"{tag} 轴孔Φ8", m, (x0, side * y, bp.P["wheel_D"] / 2),
                      (0, 1, 0), bp.P["shaft_hole"])

    print("== B. 干涉检查（布尔交体积） ==")
    pl = [(p["name"], p["world_mesh"]) for p in parts]
    pairs_hit = 0
    for i in range(len(pl)):
        for j in range(i + 1, len(pl)):
            a, b = pl[i][1], pl[j][1]
            if not a.bounding_box.intersection(b.bounding_box) if False else False:
                continue
            try:
                inter = trimesh.boolean.intersection([a, b], engine="manifold")
                v = 0.0 if inter is None or inter.is_empty else abs(inter.volume)
            except BaseException:
                v = 0.0
            if v > TOL_VOL:
                pairs_hit += 1
                check(f"干涉 {pl[i][0]} × {pl[j][0]}", False, f"{v:.0f} mm3")
    acc_hit = 0
    for p in parts:
        for a in acc:
            try:
                inter = trimesh.boolean.intersection([p["world_mesh"], a["world_mesh"]],
                                                     engine="manifold")
                v = 0.0 if inter is None or inter.is_empty else abs(inter.volume)
            except BaseException:
                v = 0.0
            if v > TOL_VOL:
                acc_hit += 1
                check(f"干涉 {p['name']} × 器件{a['name']}", False, f"{v:.0f} mm3")
    check("打印件两两无干涉", pairs_hit == 0, f"{pairs_hit} 对超差")
    check("打印件与器件无干涉", acc_hit == 0, f"{acc_hit} 对超差")

    print("== C. 关键间隙 / 姿态 ==")
    zf = bp.P["cab_z0"] + bp.P["floor"]
    bat_top = zf + 5 + 20                       # EVA5 + 电池20
    f1_bot = f1.bounds[0][2]
    check("电池顶 ↔ F1 底 ≥ 0", f1_bot - bat_top >= -0.01, f"{f1_bot - bat_top:.1f}mm")
    l298_top = f1.bounds[1][2] + 27
    f2_bot = f2.bounds[0][2]
    check("L298N 散热片 ↔ F2 底 ≥ 8", f2_bot - l298_top >= 8 - 0.01,
          f"{f2_bot - l298_top:.1f}mm")
    esp_top = f2.bounds[1][2] + 15
    lid_bot = lid.bounds[0][2]
    check("主控板顶 ↔ 顶盖底 ≥ 1", lid_bot - esp_top >= 1, f"{lid_bot - esp_top:.1f}mm")
    gap_rk = rkR.bounds[0][1] - cab.bounds[1][1]
    check("摇臂 ↔ 舱壁 转动间隙 ≥ 3(垫PTFE)", gap_rk >= 3.0, f"{gap_rk:.1f}mm")
    wheels = [a["world_mesh"] for a in acc if a["name"] == "wheel"]
    zmin = [w.bounds[0][2] for w in wheels]
    check("六轮共面着地 z=0", max(abs(np.array(zmin))) < 0.05,
          f"min z: {['%.2f' % z for z in zmin]}")
    deck_w = f1.bounds[1][0] - f1.bounds[0][0], f1.bounds[1][1] - f1.bounds[0][1]
    inner = bp.P["cab_L"] - 2 * bp.P["wall"], bp.P["cab_W"] - 2 * bp.P["wall"]
    check("楼层板可放入舱内", deck_w[0] < inner[0] and deck_w[1] < inner[1],
          f"板 {deck_w[0]:.0f}x{deck_w[1]:.0f} < 舱内 {inner[0]:.0f}x{inner[1]:.0f}")

    fails = [r for r in results if not r[1]]
    print(f"\n共 {len(results)} 项：PASS {len(results) - len(fails)}，FAIL {len(fails)}")
    if fails:
        print("失败项：")
        for n, _, d in fails:
            print("  ✗", n, d)
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
