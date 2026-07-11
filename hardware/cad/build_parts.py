#!/usr/bin/env python3
"""ARES V3 全车打印件参数化生成器（水密实体 + 全部螺孔）。

依据：
  hardware/螺丝&螺母&垫片清单.md   —— M3 电路/电机、M4 二级铰链、M5 主铰链
  docs/Ares V2.2 ... (Footprint Data).md —— 器件外形尺寸
  docs/楼层布局.md / hardware/drawings/总体设计.jpg —— 主舱 190x149x75、轮距 160、轮径 65

用法：
  pip install trimesh manifold3d numpy shapely scipy networkx
  python build_parts.py            # 生成 "Print required/" 全部 STL 与装配数据

坐标系（世界系）：X 车头方向，Y 左侧，Z 向上，地面 z=0。
每个零件先在世界位姿建模（便于装配校验），再变换到打印姿态导出。
"""
import math
from pathlib import Path

import numpy as np
import trimesh
from trimesh.transformations import rotation_matrix as rot
from trimesh.transformations import translation_matrix as tra

OUT = Path(__file__).resolve().parents[2] / "Print required"

# ============================ 参数表 ============================
P = dict(
    # 主舱（外廓 190x149 来自总体设计.jpg；高度按图上"9cm"取 90——
    # 旧 主舱.stl 的 75 装不下楼层叠高 5+25+4+35+4+主控板15=88，是错误值）
    cab_L=190.0, cab_W=149.0, cab_H=90.0, wall=4.0, floor=5.0,
    cab_z0=45.0,                      # 主舱底板离地（离地间隙）
    post=8.0,                         # 角柱边长
    # 车轮 / 电机（Footprint: TT 70x20x20，轮 Φ65x26）
    wheel_D=65.0, wheel_T=26.0,
    motor_L=70.0, motor_W=20.0, motor_H=20.0,
    motor_hole_pitch=17.6,            # TT 两个 M3 贯穿孔间距
    motor_hole_from_shaft=11.4,       # 轴心到近端安装孔
    shaft_hole=8.0,                   # 摆臂上的轴过孔
    # 悬挂几何（总体设计.jpg：轮距 160、摇臂 220-260、转向架 140-170）
    x_front=160.0, x_mid=0.0, x_rear=-160.0,
    hingeA=(40.0, 67.5),              # 主铰链 A (x, z)
    hingeB=(-75.0, 55.0),             # 二级铰链 B (x, z)
    arm_t=8.0, arm_h=20.0,            # 摆臂厚 / 梁高
    rocker_y=79.5,                    # 右摇臂内侧面 y（左侧镜像）
    bogie_gap=0.8,                    # 摇臂-转向架间 PTFE 垫片厚
    # 孔径（螺丝清单）
    m3=3.4, m3_slot=6.0, m4=4.5, m5=5.5,
    m3_pilot=2.9, m2_pilot=1.8,
    hex_m4=7.4, hex_m4_h=3.2,         # M4 螺母沉槽（对边 7.0 +0.4 余量）
    # 楼层板
    deck_L=170.0, deck_W=115.0, deck_t=4.0,
    standoff_x=75.0, standoff_y=47.5, # 支撑柱孔半距（150x95 矩形）
    # 器件安装孔
    l298_pitch=37.0,                  # L298N 4 孔 37x37 Φ3
    lm2596_holes=(30.0, 16.0),        # LM2596 对角两孔
    perf_hx=74.0, perf_hy=54.0,       # 80x60 洞洞板角孔
    # 桅杆（柱子.stl：底座 54x42、杆 18x18x150）
    mast_base=(54.0, 42.0, 5.0), mast_sq=18.0, mast_h=150.0,
    mast_holes=(44.0, 32.0),
    sg90=(23.4, 12.8, 24.0),          # SG90 顶部插槽
    # 平衡差动杆
    bar_L=200.0, bar_W=16.0, bar_t=8.0,
    horn_top_z=146.0,                 # 摇臂立柱顶面 = 顶盖(135+3)+枢轴凸台(8)
    # 超声波（HC-SR04 45x20x18，探头 Φ16 间距 26）
    sonar_hole=16.4, sonar_pitch=26.0, sonar_wing_deg=32.0,
    # 开关（Footprint：25x15x65，横装+护翼比拨杆长 5mm）
    switch_hole=12.6,
)

SEC = 64  # 圆分段


# ============================ 基础工具 ============================
def box(sx, sy, sz, at=(0, 0, 0)):
    m = trimesh.creation.box((sx, sy, sz))
    m.apply_translation(at)
    return m


def cyl(r, h, at=(0, 0, 0), axis="z"):
    m = trimesh.creation.cylinder(radius=r, height=h, sections=SEC)
    if axis == "x":
        m.apply_transform(rot(math.pi / 2, (0, 1, 0)))
    elif axis == "y":
        m.apply_transform(rot(math.pi / 2, (1, 0, 0)))
    m.apply_translation(at)
    return m


def hexp(af, h, at=(0, 0, 0), axis="y"):
    """六角柱（af=对边距），用作螺母沉槽。"""
    r = af / math.sqrt(3)
    m = trimesh.creation.cylinder(radius=r, height=h, sections=6)
    if axis == "x":
        m.apply_transform(rot(math.pi / 2, (0, 1, 0)))
    elif axis == "y":
        m.apply_transform(rot(math.pi / 2, (1, 0, 0)))
    m.apply_translation(at)
    return m


def slot(w, l, h, at=(0, 0, 0), axis="z", along="x"):
    """腰形槽：宽 w、总长 l（含两端半圆）、深 h。"""
    seg = max(l - w, 0.01)
    parts = [box(seg, w, h) if along == "x" else box(w, seg, h),
             cyl(w / 2, h, at=(seg / 2, 0, 0) if along == "x" else (0, seg / 2, 0)),
             cyl(w / 2, h, at=(-seg / 2, 0, 0) if along == "x" else (0, -seg / 2, 0))]
    m = union(parts)
    if axis == "x":
        m.apply_transform(rot(math.pi / 2, (0, 1, 0)))
    elif axis == "y":
        m.apply_transform(rot(math.pi / 2, (1, 0, 0)))
    m.apply_translation(at)
    return m


def bar2d(p1, p2, height, thick, y_center):
    """XZ 平面内从 p1 到 p2 的梁（截面 height x thick），厚度沿 Y。"""
    (x1, z1), (x2, z2) = p1, p2
    L = math.hypot(x2 - x1, z2 - z1)
    ang = math.atan2(z2 - z1, x2 - x1)
    m = box(L + 0.02, thick, height)
    m.apply_transform(rot(-ang, (0, 1, 0)))  # 绕 Y 旋转到段方向
    m.apply_translation(((x1 + x2) / 2, y_center, (z1 + z2) / 2))
    return m


def union(meshes):
    meshes = [m for m in meshes if m is not None]
    return trimesh.boolean.union(meshes, engine="manifold") if len(meshes) > 1 else meshes[0]


def cut(solid, cutters):
    return trimesh.boolean.difference([solid] + list(cutters), engine="manifold")


def mirror_y(mesh):
    m = mesh.copy()
    m.apply_transform(np.diag([1.0, -1.0, 1.0, 1.0]))
    if m.volume < 0:          # trimesh 对反射矩阵通常已自动翻面，仅在异常时补翻
        m.invert()
    return m


def teardrop(r, h, at, axis="x"):
    """水平孔用泪滴形（顶部 45° 收尖，免支撑打印）。"""
    c = trimesh.creation.cylinder(radius=r, height=h, sections=SEC)
    tip = trimesh.creation.box((r * 1.414, r * 1.414, h))
    tip.apply_transform(rot(math.pi / 4, (0, 0, 1)))
    tip.apply_translation((0, r * 0.9, 0))
    m = union([c, tip])
    if axis == "x":
        m.apply_transform(rot(math.pi / 2, (0, 1, 0)))
    elif axis == "y":
        m.apply_transform(rot(math.pi / 2, (1, 0, 0)))
    m.apply_translation(at)
    return m


# ============================ 零件：主舱 ============================
def cabin():
    L, W, H, w, f = P["cab_L"], P["cab_W"], P["cab_H"], P["wall"], P["floor"]
    hz = H - f  # 墙高
    body = [box(L, W, f, (0, 0, f / 2))]
    body += [box(L, w, hz, (0, +(W - w) / 2, f + hz / 2)),
             box(L, w, hz, (0, -(W - w) / 2, f + hz / 2)),
             box(w, W - 2 * w, hz, (+(L - w) / 2, 0, f + hz / 2)),
             box(w, W - 2 * w, hz, (-(L - w) / 2, 0, f + hz / 2))]
    # 角柱（对称！修复原 STL 的 +74/-86 不对称缺陷）
    p = P["post"]
    px, py = L / 2 - w - p / 2, W / 2 - w - p / 2
    for sx in (1, -1):
        for sy in (1, -1):
            body.append(box(p, p, hz, (sx * px, sy * py, f + hz / 2)))
    # 主铰链 A 舱内加厚凸台（Φ22x8 向内，摇臂贴外墙平面转动，轴承长 12mm）
    # 凸台下方连到底板的支撑柱：打印时自下而上成形，零支撑
    ax, az = P["hingeA"][0], P["hingeA"][1] - P["cab_z0"]
    boss_len = 8.0
    for sy in (1, -1):
        yc = sy * (W / 2 - w - boss_len / 2)
        body.append(cyl(11.0, boss_len, (ax, yc, az), axis="y"))
        body.append(box(18.0, boss_len, az - f, (ax, yc, f + (az - f) / 2)))
    solid = union(body)

    cuts = []
    # 主铰链 A 通孔 M5（贯穿两侧墙+凸台）
    cuts.append(cyl(P["m5"] / 2, W + 40, (ax, 0, az), axis="y"))
    # 底板：4 个支撑柱孔 M3 + 底部沉头
    for sx in (1, -1):
        for sy in (1, -1):
            x, y = sx * P["standoff_x"], sy * P["standoff_y"]
            cuts.append(cyl(P["m3"] / 2, 30, (x, y, f / 2)))
            cuts.append(cyl(3.4, 2.2, (x, y, 1.1)))
    # 电池魔术贴槽 4 条（电池 75x40 居中）
    for sx in (1, -1):
        for sy in (1, -1):
            cuts.append(slot(4.0, 22.0, 30, (sx * 25, sy * 33, f / 2), along="y"))
    # 角柱顶 M3 底孔（自攻，深 12）
    for sx in (1, -1):
        for sy in (1, -1):
            cuts.append(cyl(P["m3_pilot"] / 2, 12.5, (sx * px, sy * py, H - 6)))
    # 前墙：超声波支架安装 2xM3（局部 z=32 → 世界 77）+ 走线孔 Φ8
    zf = 32.0
    for sy in (1, -1):
        cuts.append(cyl(P["m3"] / 2, 20, (L / 2 - w / 2, sy * 20, zf), axis="x"))
    cuts.append(cyl(4.0, 20, (L / 2 - w / 2, 0, zf + 10), axis="x"))
    # 后墙：钮子开关 Φ12.6（横装，B1 层高度，避开 F1 楼层板）+ 护架 2xM3 + 充电口 Φ8 + 散热孔
    zs = 19.0
    cuts.append(cyl(P["switch_hole"] / 2, 20, (-(L / 2 - w / 2), 30, zs), axis="x"))
    for dy in (14, -14):
        cuts.append(cyl(P["m3"] / 2, 20, (-(L / 2 - w / 2), 30 + dy, zs + 0), axis="x"))
    cuts.append(cyl(4.0, 20, (-(L / 2 - w / 2), -35, 20), axis="x"))
    for dy in (-10, 0, 10):
        cuts.append(cyl(3.0, 20, (-(L / 2 - w / 2), dy - 0, 58), axis="x"))
        cuts.append(cyl(3.0, 20, (L / 2 - w / 2, dy, 62), axis="x"))
    solid = cut(solid, cuts)
    world = tra((0, 0, P["cab_z0"]))
    return dict(name="01_主舱_cabin", mesh=solid, world=world, color=(200, 205, 215),
                note="含主铰链A M5 凸台、支撑柱孔、开关/充电/超声波开孔")


# ============================ 零件：顶盖 ============================
def lid():
    L, W, w = P["cab_L"], P["cab_W"], P["wall"]
    t = 3.0
    body = [box(L, W, t, (0, 0, t / 2))]
    # 平衡杆枢轴凸台（在主铰链 A 正上方 x=40）
    bx = P["hingeA"][0]
    body.append(cyl(8.0, 8.0, (bx, 0, t + 4.0)))
    solid = union(body)
    cuts = []
    # 4 角与主舱角柱对位（修复原 STL 不对位缺陷）
    px, py = L / 2 - w - P["post"] / 2, W / 2 - w - P["post"] / 2
    for sx in (1, -1):
        for sy in (1, -1):
            cuts.append(cyl(P["m3"] / 2, 20, (sx * px, sy * py, t / 2)))
    # 桅杆 4xM3（44x32）+ 中心走线 Φ10
    for sx in (1, -1):
        for sy in (1, -1):
            cuts.append(cyl(P["m3"] / 2, 20, (sx * P["mast_holes"][0] / 2,
                                              sy * P["mast_holes"][1] / 2, t / 2)))
    cuts.append(cyl(5.0, 20, (0, 0, t / 2)))
    # 平衡杆 M5 枢轴孔（贯穿凸台）
    cuts.append(cyl(P["m5"] / 2, 40, (bx, 0, t + 4)))
    # 甲虫穹顶碳杆锚孔 Φ6 x4（角柱内侧）
    for sx in (1, -1):
        for sy in (1, -1):
            cuts.append(cyl(3.0, 20, (sx * (px - 14), sy * (py - 10), t / 2)))
    solid = cut(solid, cuts)
    world = tra((0, 0, P["cab_z0"] + P["cab_H"]))
    return dict(name="02_顶盖_lid", mesh=solid, world=world, color=(170, 178, 192),
                note="角孔与主舱角柱严格对位；桅杆孔 44x32；平衡杆枢轴凸台")


# ============================ 摆臂公共：电机站 ============================
def motor_station(x, z, y_inner, body_dir, side):
    """返回 (加强盘 list, 切割 list)。body_dir=+1 电机体朝 +X。side=+1 右侧。
    电机贴摆臂内侧面，轴穿 Φ8 孔；臂外侧沉台 Φ24 使局部仅剩 3mm，保证轴伸出量。"""
    t, y_c = P["arm_t"], y_inner + P["arm_t"] / 2 * 0  # 计算用
    yc = y_inner + P["arm_t"] / 2 if side > 0 else y_inner - P["arm_t"] / 2
    adds = [cyl(15.0, t, (x, yc, z), axis="y")]
    cuts = [cyl(P["shaft_hole"] / 2, t + 4, (x, yc, z), axis="y")]
    # 外侧沉台（留 3mm 壁）
    rec_d = t - 3.0
    y_rec = yc + side * (t / 2 - rec_d / 2 + 0.01)
    cuts.append(cyl(12.0, rec_d, (x, y_rec, z), axis="y"))
    # 2 x M3x30 贯穿槽孔（竖向腰形，吸收电机孔位公差）。
    # 电机主固定 = 螺丝清单的 M3x30 贯穿螺栓；扎带可绕臂整体捆扎，不再开槽
    # （原扎带槽距轴孔仅 6mm，会与 M3 槽/轴孔在 X 向串通成 18mm 大槽掏空腹板）
    for k in (1, 2):
        dx = body_dir * (P["motor_hole_from_shaft"] + (k - 1) * P["motor_hole_pitch"])
        adds.append(cyl(6.5, t, (x + dx, yc, z), axis="y"))
        cuts.append(slot(P["m3"] + 0.2, P["m3_slot"], t + 6, (x + dx, yc, z),
                         axis="y", along="x"))
    return adds, cuts


# ============================ 零件：摇臂（右） ============================
def rocker_R():
    (ax, az), (bx, bz) = P["hingeA"], P["hingeB"]
    fx, fz = P["x_front"], P["wheel_D"] / 2
    y_in = P["rocker_y"]
    yc = y_in + P["arm_t"] / 2
    h, t = P["arm_h"], P["arm_t"]
    body = [bar2d((ax, az), (fx, fz), h, t, yc),
            bar2d((ax, az), (bx, bz), h, t, yc),
            cyl(10.0, t, (ax, yc, az), axis="y"),      # 铰链 A 盘
            cyl(9.0, t, (bx, yc, bz), axis="y")]       # 铰链 B 盘
    # 差动立柱（顶面 z=horn_top_z，顶板带 M4 竖孔）
    horn_h = P["horn_top_z"] - az
    body.append(box(12, t, horn_h, (ax, yc, az + horn_h / 2)))
    body.append(box(20, t, 8, (ax, yc, P["horn_top_z"] - 4)))
    adds, cuts = motor_station(fx, fz, y_in, body_dir=-1, side=+1)
    body += adds
    cuts.append(cyl(P["m5"] / 2, t + 6, (ax, yc, az), axis="y"))       # A: M5
    cuts.append(cyl(P["m4"] / 2, t + 6, (bx, yc, bz), axis="y"))       # B: M4
    cuts.append(cyl(P["m4"] / 2, 20, (ax, yc, P["horn_top_z"] - 4)))   # 立柱顶 M4 竖孔
    solid = cut(union(body), cuts)
    return dict(name="05_右摇臂_rocker_R", mesh=solid, world=np.eye(4),
                color=(235, 160, 80),
                note="A孔Φ5.5 B孔Φ4.5 前电机站(Φ8轴孔+2xM3槽+沉台) 差动立柱M4")


# ============================ 零件：转向架（右） ============================
def bogie_R():
    bx, bz = P["hingeB"]
    mx, rx, wz = P["x_mid"], P["x_rear"], P["wheel_D"] / 2
    t, h = P["arm_t"], P["arm_h"]
    y_in = P["rocker_y"] + P["arm_t"] + P["bogie_gap"]
    yc = y_in + t / 2
    body = [bar2d((bx, bz), (mx, wz), h, t, yc),
            bar2d((bx, bz), (rx, wz), h, t, yc),
            # 中轮站前伸短梁：中轮电机体朝 +X，远端 M3 贯穿孔需要着力材料
            box(40.0, t, h, (mx + 20.0, yc, wz)),
            cyl(9.0, t, (bx, yc, bz), axis="y")]
    a1, c1 = motor_station(mx, wz, y_in, body_dir=+1, side=+1)
    a2, c2 = motor_station(rx, wz, y_in, body_dir=+1, side=+1)
    body += a1 + a2
    cuts = c1 + c2
    cuts.append(cyl(P["m4"] / 2, t + 6, (bx, yc, bz), axis="y"))
    # 外侧 M4 螺母六角沉槽
    cuts.append(hexp(P["hex_m4"], P["hex_m4_h"],
                     (bx, yc + t / 2 - P["hex_m4_h"] / 2 + 0.01, bz), axis="y"))
    solid = cut(union(body), cuts)
    return dict(name="07_右转向架_bogie_R", mesh=solid, world=np.eye(4),
                color=(120, 200, 140),
                note="B孔Φ4.5+螺母沉槽 中/后电机站x2")


# ============================ 零件：楼层板 ============================
def deck(which):
    L, W, t = P["deck_L"], P["deck_W"], P["deck_t"]
    solid = box(L, W, t, (0, 0, t / 2))
    cuts = []
    for sx in (1, -1):                                   # 支撑柱孔
        for sy in (1, -1):
            cuts.append(cyl(P["m3"] / 2, 20, (sx * P["standoff_x"], sy * P["standoff_y"], t / 2)))
    cuts.append(slot(14.0, 30.0, 20, (L / 2, 0, t / 2), along="y"))   # 前缘走线口
    if which == "F1":
        # 后缘：y=-30 走线口 + y=+30 开关避让缺口（钮子开关本体 40x15x25 在 B1 层后部）
        cuts.append(slot(14.0, 26.0, 20, (-L / 2, -30, t / 2), along="y"))
        cuts.append(box(42.0, 26.0, 20, (-L / 2 + 17, 30, t / 2)))                                    # 2x L298N + LM2596
        for sy in (1, -1):
            for dx in (1, -1):
                for dy in (1, -1):
                    cuts.append(cyl(1.7, 20, (dx * P["l298_pitch"] / 2,
                                              sy * 32.5 + dy * P["l298_pitch"] / 2, t / 2)))
        for k in (1, -1):
            cuts.append(cyl(1.7, 20, (45 + k * P["lm2596_holes"][0] / 2,
                                      k * P["lm2596_holes"][1] / 2, t / 2)))
        z_world = P["cab_z0"] + P["floor"] + 25.0
        name, note = "03_楼层F1_deck", "L298N 37x37 孔x2 组、LM2596 孔、支撑柱 150x95"
    else:                                                # F2：80x60 洞洞板槽孔
        cuts.append(slot(14.0, 30.0, 20, (-L / 2, 0, t / 2), along="y"))
        for sx in (1, -1):
            for sy in (1, -1):
                cuts.append(slot(P["m3"], 7.0, 20, (sx * P["perf_hx"] / 2,
                                                    sy * P["perf_hy"] / 2, t / 2), along="x"))
        z_world = P["cab_z0"] + P["floor"] + 25.0 + P["deck_t"] + 35.0
        name, note = "04_楼层F2_deck", "洞洞板 74x54 腰形孔（容差）、支撑柱 150x95"
    solid = cut(solid, cuts)
    return dict(name=name, mesh=solid, world=tra((0, 0, z_world)),
                color=(150, 170, 210), note=note)


# ============================ 零件：平衡差动杆 ============================
def balance_bar():
    Lb, Wb, t = P["bar_L"], P["bar_W"], P["bar_t"]
    yh = P["rocker_y"] + P["arm_t"] / 2                 # 立柱中心面（杆建模即沿 Y）
    solid = union([box(Wb, Lb, t, (0, 0, t / 2)), cyl(9.0, t, (0, 0, t / 2)),
                   cyl(9.5, t, (0, yh, t / 2)), cyl(9.5, t, (0, -yh, t / 2))])
    cuts = [cyl(P["m5"] / 2, 20, (0, 0, t / 2))]
    for s in (1, -1):
        cuts.append(slot(P["m4"] + 0.2, 11.0, 20, (0, s * yh, t / 2), along="x"))
    solid = cut(solid, cuts)
    world = tra((P["hingeA"][0], 0, P["horn_top_z"]))
    return dict(name="09_平衡差动杆_balance_bar", mesh=solid, world=world,
                color=(230, 120, 120),
                note="中心Φ5.5 接顶盖枢轴；两端腰形孔 M4 接左右摇臂立柱")


# ============================ 零件：桅杆 ============================
def mast():
    bw, bd, bt = P["mast_base"]
    sq, mh = P["mast_sq"], P["mast_h"]
    head_w, head_d, head_h = 34.0, 20.0, 28.0
    body = [box(bw, bd, bt, (0, 0, bt / 2)),
            box(sq, sq, mh, (0, 0, bt + mh / 2)),
            box(head_w, head_d, head_h, (0, 0, bt + mh + head_h / 2))]
    for s in (1, -1):                                    # 三角加强筋 x2
        g = box(14.0, 4.0, 14.0, (s * (sq / 2 + 7), 0, bt + 7))
        gc = trimesh.creation.box((22, 6, 22))
        gc.apply_transform(rot(math.pi / 4, (0, 1, 0)))
        gc.apply_translation((s * (sq / 2 + 14.5), 0, bt + 14.5))
        body.append(cut(g, [gc]))
    solid = union(body)
    cuts = []
    for sx in (1, -1):                                   # 底座 4xM3
        for sy in (1, -1):
            cuts.append(cyl(P["m3"] / 2, 20, (sx * P["mast_holes"][0] / 2,
                                              sy * P["mast_holes"][1] / 2, bt / 2)))
    cuts.append(cyl(5.0, 20, (0, 0, bt / 2)))            # 中心走线
    sw, sd, sdep = P["sg90"]                             # SG90 顶部插槽（上开口）
    ztop = bt + mh + head_h
    cuts.append(box(sw, sd, sdep + 0.1, (0, 0, ztop - sdep / 2)))
    for s in (1, -1):                                    # 舵机耳片 M2 自攻竖孔
        cuts.append(cyl(P["m2_pilot"] / 2, 12, (s * (sw / 2 + 2.3), 0, ztop - 5)))
    for dz in (30, 75, 120):                             # 走线扎带槽
        cuts.append(slot(3.2, 12.0, sq + 6, (0, 0, bt + dz), axis="y", along="x"))
    solid = cut(solid, cuts)
    world = tra((0, 0, P["cab_z0"] + P["cab_H"] + 3.0))
    return dict(name="10_桅杆_mast", mesh=solid, world=world, color=(240, 200, 90),
                note="底座4xM3+走线孔 SG90插槽+M2耳孔 扎带槽x3；侧躺打印")


# ============================ 零件：相机支架 ============================
def cam_bracket():
    bp = box(30.0, 3.0, 34.0, (0, 1.5, 17))              # 背板
    rails = [box(3.0, 8.0, 34.0, (s * 15.3, -4 + 1.5 + 0.5, 17)) for s in (1, -1)]
    ledge = box(30.0, 8.0, 3.0, (0, -2.0, 1.5))
    base = box(24.0, 12.0, 3.0, (0, 7.5, -1.5 + 0.01))   # 舵机盘接口底板
    solid = union([bp, *rails, ledge, base])
    cuts = [cyl(1.0, 10, (0, 7.5, -1.5)),                # 舵机十字盘中心
            *[cyl(1.0, 10, (dx, 7.5, -1.5)) for dx in (7, -7)],
            slot(3.2, 10.0, 12, (0, 1.5, 30), axis="y", along="x")]  # 扎带槽
    solid = cut(solid, cuts)
    world = tra((0, 0, P["cab_z0"] + P["cab_H"] + 3 + P["mast_base"][2] + P["mast_h"] + 28 + 6))
    return dict(name="11_相机支架_cam_bracket", mesh=solid, world=world,
                color=(240, 200, 90), note="ESP32-CAM 27.4 卡槽 + 舵机盘 Φ2 孔 + 扎带槽")


# ============================ 零件：开关护架 ============================
def switch_guard():
    back = box(56.0, 4.0, 24.0, (0, 2, 12))
    wings = [box(4.0, 48.0, 24.0, (s * 26, -22, 12)) for s in (1, -1)]
    solid = union([back, *wings])
    cuts = [cyl(P["switch_hole"] / 2, 12, (0, 2, 12), axis="y")]
    for dx in (14, -14):
        cuts.append(cyl(P["m3"] / 2, 12, (dx, 2, 12), axis="y"))
    solid = cut(solid, cuts)
    # 背板外面(local y=4)贴后墙外面(x=-95)，护翼朝 -X 包住拨杆
    w = tra((-(P["cab_L"] / 2) - 4.0, 30, P["cab_z0"] + 19 - 12)) @ rot(-math.pi / 2, (0, 0, 1))
    return dict(name="12_开关护架_switch_guard", mesh=solid, world=w,
                color=(230, 120, 120),
                note="Φ12.6 开关孔与主舱后墙对位；护翼 48 长（比拨杆长≥5）")


# ============================ 零件：超声波支架 ============================
def sonar_bracket():
    t, hgt, seg = 3.5, 24.0, 52.0
    ang = math.radians(P["sonar_wing_deg"])
    center = box(seg, t, hgt, (0, -t / 2, hgt / 2))
    wings = []
    for s in (1, -1):
        wgs = box(seg, t, hgt, (0, -t / 2, hgt / 2))
        wgs.apply_transform(rot(-s * ang, (0, 0, 1)))
        dx = seg / 2 + (seg / 2) * math.cos(ang)
        dy = -(seg / 2) * math.sin(ang)
        wgs.apply_translation((s * dx, dy, 0))
        wings.append(wgs)
    # 安装耳：与背板共面向上延伸，孔轴沿 local y（世界 X，正对前墙螺孔）
    ears = [box(14.0, t, 16.0, (s * 20, -t / 2, hgt + 8 - 0.1)) for s in (1, -1)]
    bridge = box(54.0, t, 6.0, (0, -t / 2, hgt + 3 - 0.1))
    solid = union([center, *wings, *ears, bridge])
    cuts = []
    # 中间站位：两个 Φ16.4 泪滴孔 + 扎带槽
    for dx in (P["sonar_pitch"] / 2, -P["sonar_pitch"] / 2):
        cuts.append(teardrop(P["sonar_hole"] / 2, 12, (dx, 0, hgt / 2), axis="y"))
    cuts.append(slot(3.2, 10, 12, (0, 0, hgt - 5), axis="y", along="x"))
    # 两翼站位：沿翼中线走 d = 翼半长 ± 探头半距
    for s in (1, -1):
        for k in (1, -1):
            d = seg / 2 + k * P["sonar_pitch"] / 2
            px = s * (seg / 2 + d * math.cos(ang))
            py = -d * math.sin(ang)
            td = teardrop(P["sonar_hole"] / 2, 14, (0, 0, 0), axis="y")
            td.apply_transform(rot(-s * ang, (0, 0, 1)))
            td.apply_translation((px, py, hgt / 2))
            cuts.append(td)
    for s in (1, -1):                                    # 安装耳 M3 孔（轴向 local y）
        cuts.append(cyl(P["m3"] / 2, 12, (s * 20, -t / 2, hgt + 8), axis="y"))
    solid = cut(solid, cuts)
    # local: x→世界 y、-y→世界 +x（前向）、z→世界 z；背面(y=0)贴前墙外面(x=95)
    # 安装耳孔 (±20, ·, hgt+8) → 世界 (95, ∓20, 60) 对准前墙 M3 孔
    w = tra((P["cab_L"] / 2, 0, P["cab_z0"] + 32 - (hgt + 8))) @ rot(math.pi / 2, (0, 0, 1))
    return dict(name="13_超声波支架_sonar_bracket", mesh=solid, world=w,
                color=(120, 200, 140),
                note="前直视+左右 32° 三站位，Φ16.4 泪滴孔，安装耳 M3 对准前墙")


# ============================ 装配配件（非打印，仅预览） ============================
def accessories():
    acc = []
    wz = P["wheel_D"] / 2

    def wheel(x, y):
        m = cyl(P["wheel_D"] / 2, P["wheel_T"], (x, y, wz), axis="y")
        return dict(name="wheel", mesh=m, world=np.eye(4), color=(60, 60, 65))

    def motor(x, y_face, body_dir, side):
        yc = y_face - side * P["motor_W"] / 2
        m = box(P["motor_L"], P["motor_W"], P["motor_H"],
                (x + body_dir * (P["motor_L"] / 2 - 11.4), yc, wz))
        sh = cyl(2.7, 34, (x, yc + side * 17, wz), axis="y")
        return dict(name="motor", mesh=union([m, sh]), world=np.eye(4),
                    color=(250, 220, 60))

    y_rocker_in = P["rocker_y"]
    y_bogie_in = P["rocker_y"] + P["arm_t"] + P["bogie_gap"]
    for s in (1, -1):
        # 前轮在摇臂外侧，中后轮在转向架外侧
        yr_out = s * (y_rocker_in + P["arm_t"])
        yb_out = s * (y_bogie_in + P["arm_t"])
        acc.append(wheel(P["x_front"], yr_out + s * P["wheel_T"] / 2 + s * 1))
        acc.append(wheel(P["x_mid"], yb_out + s * P["wheel_T"] / 2 + s * 1))
        acc.append(wheel(P["x_rear"], yb_out + s * P["wheel_T"] / 2 + s * 1))
        acc.append(motor(P["x_front"], s * y_rocker_in, -1, s))
        acc.append(motor(P["x_mid"], s * y_bogie_in, +1, s))
        acc.append(motor(P["x_rear"], s * y_bogie_in, +1, s))
    # 舱内器件
    z0 = P["cab_z0"] + P["floor"]
    acc.append(dict(name="battery", mesh=box(75, 40, 20, (0, 0, z0 + 10 + 5)),
                    world=np.eye(4), color=(90, 140, 220)))
    zf1 = z0 + 25 + P["deck_t"]
    for sy in (1, -1):
        acc.append(dict(name="L298N", mesh=box(43, 43, 27, (0, sy * 32.5, zf1 + 13.5)),
                        world=np.eye(4), color=(200, 60, 60)))
    acc.append(dict(name="LM2596", mesh=box(43, 21, 14, (45, 0, zf1 + 7)),
                    world=np.eye(4), color=(60, 130, 200)))
    zf2 = zf1 + 35 + P["deck_t"]
    acc.append(dict(name="ESP32板", mesh=box(80, 60, 15, (0, 0, zf2 + 7.5)),
                    world=np.eye(4), color=(50, 90, 60)))
    # 桅杆顶 SG90 + 相机
    zm = P["cab_z0"] + P["cab_H"] + 3 + P["mast_base"][2] + P["mast_h"]
    acc.append(dict(name="SG90", mesh=box(23, 12.5, 26, (0, 0, zm + 28 - 11)),
                    world=np.eye(4), color=(70, 120, 240)))
    zc = P["cab_z0"] + P["cab_H"] + 3 + P["mast_base"][2] + P["mast_h"] + 28 + 6
    acc.append(dict(name="ESP32-CAM", mesh=box(27, 4, 38, (0, -2.5, 22)),
                    world=tra((0, 0, zc)), color=(40, 40, 45)))
    # 3x HC-SR04：在支架局部系里贴每个站位前面，再用支架的世界变换（保证对位）
    t, hgt, seg = 3.5, 24.0, 52.0
    ang = math.radians(P["sonar_wing_deg"])
    w_bracket = tra((P["cab_L"] / 2, 0, P["cab_z0"] + 32 - (hgt + 8))) @ rot(math.pi / 2, (0, 0, 1))
    for s in (0, 1, -1):
        m = box(45, 18, 20)   # 板 45 宽、探头向前 18 厚示意、20 高
        o = t / 2 + 9 + 1.0        # 板半厚 + 盒半深(18/2) + 1mm 间隙
        if s == 0:
            m.apply_translation((0, -t / 2 - o, hgt / 2))
        else:
            m.apply_transform(rot(-s * ang, (0, 0, 1)))
            # 翼板真实中线：起点(s·seg/2, -t/2)，方向(s·cosα, -sinα)，取中点
            midx = s * seg / 2 + s * math.cos(ang) * seg / 2
            midy = -t / 2 - math.sin(ang) * seg / 2
            n = (-s * math.sin(ang), -math.cos(ang))
            m.apply_translation((midx + n[0] * o, midy + n[1] * o, hgt / 2))
        acc.append(dict(name="HC-SR04", mesh=m, world=w_bracket, color=(80, 160, 230)))
    # 开关：本体在舱内 B1 层，拨杆穿后墙伸出（护翼必须比拨杆长）
    body = box(40, 15, 25, (-(P["cab_L"] / 2) + 4 + 20, 30, P["cab_z0"] + 19))
    lever = cyl(2.0, 35, (-(P["cab_L"] / 2) - 17.5, 30, P["cab_z0"] + 19), axis="x")
    acc.append(dict(name="switch", mesh=union([body, lever]), world=np.eye(4),
                    color=(20, 20, 22)))
    return acc


# ============================ 导出与自检 ============================
PRINT_ORIENT = {   # 每个零件的打印姿态（把世界系建模件转到 Z 朝上平躺）
    "05_右摇臂_rocker_R":  lambda m: m.apply_transform(rot(math.pi / 2, (1, 0, 0))),
    "06_左摇臂_rocker_L":  lambda m: m.apply_transform(rot(-math.pi / 2, (1, 0, 0))),
    "07_右转向架_bogie_R": lambda m: m.apply_transform(rot(math.pi / 2, (1, 0, 0))),
    "08_左转向架_bogie_L": lambda m: m.apply_transform(rot(-math.pi / 2, (1, 0, 0))),
    "10_桅杆_mast":        lambda m: m.apply_transform(rot(-math.pi / 2, (0, 1, 0))),
    "12_开关护架_switch_guard": lambda m: m.apply_transform(rot(-math.pi / 2, (1, 0, 0))),
}


def build_all():
    parts = [cabin(), lid(), deck("F1"), deck("F2")]
    rr = rocker_R()
    parts.append(rr)
    rl = dict(rr, name="06_左摇臂_rocker_L", mesh=mirror_y(rr["mesh"]))
    parts.append(rl)
    br = bogie_R()
    parts.append(br)
    bl = dict(br, name="08_左转向架_bogie_L", mesh=mirror_y(br["mesh"]))
    parts.append(bl)
    parts += [balance_bar(), mast(), cam_bracket(), switch_guard(), sonar_bracket()]
    return parts


def fit_bed(m, limit=245.0):
    """平面零件绕 Z 旋转寻优，使 XY 包络进入打印床（对角摆放）。"""
    best, best_ang = None, 0.0
    for deg in range(0, 180, 2):
        mm = m.copy()
        mm.apply_transform(rot(math.radians(deg), (0, 0, 1)))
        e = mm.bounding_box.extents
        key = max(e[0], e[1])
        if best is None or key < best:
            best, best_ang = key, deg
        if key <= limit and deg == 0:
            return m, 0.0
    mm = m.copy()
    mm.apply_transform(rot(math.radians(best_ang), (0, 0, 1)))
    return mm, best_ang


def export(parts):
    pdir = OUT / "parts"
    pdir.mkdir(parents=True, exist_ok=True)
    report = []
    for p in parts:
        m = p["mesh"].copy()
        if p["name"] in PRINT_ORIENT:
            PRINT_ORIENT[p["name"]](m)
        m, ang = fit_bed(m)
        p["print_rot_deg"] = ang
        m.rezero()
        p["mesh_print"] = m
        ext = m.bounding_box.extents
        ok_wt = m.is_watertight
        ok_size = max(ext[0], ext[1]) <= 246.0 and ext[2] <= 250.0
        report.append((p["name"], ext, m.volume / 1000, ok_wt, ok_size, ang))
        m.export(pdir / f"{p['name']}.stl")
    print(f"{'零件':34s} {'打印尺寸 (mm)':24s} {'体积cm3':>8s} 水密 ≤床 斜放°")
    for n, e, v, wt, sz, ang in report:
        print(f"{n:34s} {e[0]:6.1f} x {e[1]:6.1f} x {e[2]:5.1f}   {v:8.1f}   "
              f"{'✓' if wt else '✗✗'}  {'✓' if sz else '✗✗'}  {ang:.0f}")
    return report


if __name__ == "__main__":
    parts = build_all()
    rep = export(parts)
    bad = [r for r in rep if not (r[3] and r[4])]
    print("\n结果:", "全部水密且 ≤250mm ✓" if not bad else f"{len(bad)} 个零件不合格 ✗")
