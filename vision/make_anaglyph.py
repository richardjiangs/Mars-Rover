#!/usr/bin/env python3
"""ARES 红蓝 3D 合成：把"伪 3D"任务拍的左右眼两张照片合成红蓝立体图。

用法:
    python make_anaglyph.py incoming/stereo_001_left.jpg incoming/stereo_001_right.jpg -o 3d.jpg
    python make_anaglyph.py --auto incoming/     # 自动配对目录里的 stereo_*_left/right

原理（docs/Ares_火星车原型制作完整指南.md 步骤 4）：车先拍左眼照，直线前进 6cm
再拍右眼照，两视角构成双目视差；红通道取左眼、青(绿+蓝)通道取右眼，戴红蓝眼镜观看。
--align 用 ORB 特征做自动对齐（相机若有轻微上下偏移时开启）。
依赖: pip install opencv-python numpy
"""
import argparse
import glob
import re
import sys
from pathlib import Path

import cv2
import numpy as np


def align(left, right):
    """用 ORB 特征估计平移，把右眼图对齐到左眼图（只补偿上下错位与小旋转）。"""
    orb = cv2.ORB_create(800)
    k1, d1 = orb.detectAndCompute(cv2.cvtColor(left, cv2.COLOR_BGR2GRAY), None)
    k2, d2 = orb.detectAndCompute(cv2.cvtColor(right, cv2.COLOR_BGR2GRAY), None)
    if d1 is None or d2 is None:
        return right
    matches = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(d1, d2)
    if len(matches) < 12:
        return right
    matches = sorted(matches, key=lambda m: m.distance)[:60]
    src = np.float32([k2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
    dst = np.float32([k1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    mat, _ = cv2.estimateAffinePartial2D(src, dst, method=cv2.RANSAC)
    if mat is None:
        return right
    mat[0, 2] = 0  # 保留水平视差（这就是 3D 感的来源），只修垂直/旋转
    return cv2.warpAffine(right, mat, (left.shape[1], left.shape[0]))


def anaglyph(left, right, do_align):
    h = min(left.shape[0], right.shape[0])
    w = min(left.shape[1], right.shape[1])
    left, right = left[:h, :w], right[:h, :w]
    if do_align:
        right = align(left, right)
    out = right.copy()          # BGR：B、G 取右眼
    out[:, :, 2] = left[:, :, 2]  # R 取左眼
    return out


def auto_pairs(folder):
    files = glob.glob(str(Path(folder) / "stereo_*_left*.jpg"))
    pairs = []
    for lf in sorted(files):
        rf = re.sub(r"_left", "_right", lf)
        if Path(rf).exists():
            pairs.append((lf, rf))
    return pairs


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("inputs", nargs="+", help="left.jpg right.jpg，或 --auto 模式给目录")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--auto", action="store_true", help="自动配对目录内 stereo_*_left/right")
    ap.add_argument("--align", action="store_true", help="ORB 特征自动对齐（修垂直错位）")
    args = ap.parse_args()

    if args.auto:
        pairs = auto_pairs(args.inputs[0])
        if not pairs:
            sys.exit("目录里没找到 stereo_*_left/right 配对")
    else:
        if len(args.inputs) != 2:
            sys.exit("需要两个文件：左眼图 右眼图（或用 --auto 目录）")
        pairs = [(args.inputs[0], args.inputs[1])]

    for lf, rf in pairs:
        left, right = cv2.imread(lf), cv2.imread(rf)
        if left is None or right is None:
            print(f"跳过（读不出图像）: {lf} / {rf}")
            continue
        out = args.out if (args.out and len(pairs) == 1) else \
            re.sub(r"_left.*\.jpg$", "_3d.jpg", lf)
        cv2.imwrite(out, anaglyph(left, right, args.align))
        print(f"合成 {out}（戴红蓝眼镜观看，红片在左眼）")


if __name__ == "__main__":
    main()
