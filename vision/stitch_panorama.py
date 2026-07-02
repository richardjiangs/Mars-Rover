#!/usr/bin/env python3
"""ARES 全景拼接：把一组全景照片拼成一张 360°/180° 全景图。

用法:
    python stitch_panorama.py incoming/pano_*.jpg -o panorama.jpg
    python stitch_panorama.py incoming/            # 目录内所有 pano_*.jpg

照片来自"拍 360° 全景"任务（舵机每 30° 一张，文件名 pano_序号_deg角度.jpg，
按文件名排序即按角度排序）。相邻照片需有约 30% 重叠才能拼接成功；
OpenCV 拼接失败时自动退化为按顺序横向排列（保底出图）。
依赖: pip install opencv-python numpy
"""
import argparse
import glob
import sys
from pathlib import Path

import cv2
import numpy as np


def load_images(inputs):
    files = []
    for item in inputs:
        p = Path(item)
        if p.is_dir():
            files += sorted(glob.glob(str(p / "pano_*.jpg"))) or sorted(glob.glob(str(p / "*.jpg")))
        else:
            files += sorted(glob.glob(item))
    files = list(dict.fromkeys(files))  # 去重保序
    imgs = []
    for f in files:
        im = cv2.imread(f)
        if im is None:
            print(f"跳过（读不出图像）: {f}")
            continue
        # 统一缩到高 720，拼接更快更稳
        if im.shape[0] > 720:
            s = 720 / im.shape[0]
            im = cv2.resize(im, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        imgs.append(im)
        print(f"载入 {f}  {im.shape[1]}x{im.shape[0]}")
    return imgs


def stitch(imgs):
    stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
    status, pano = stitcher.stitch(imgs)
    if status == cv2.Stitcher_OK:
        return pano, True
    print(f"OpenCV 拼接失败（status={status}，常见原因：重叠不足/画面无纹理），"
          "退化为按顺序横向排列")
    h = min(im.shape[0] for im in imgs)
    row = [cv2.resize(im, (int(im.shape[1] * h / im.shape[0]), h)) for im in imgs]
    return np.hstack(row), False


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("inputs", nargs="+", help="照片文件/通配符/目录")
    ap.add_argument("-o", "--out", default="panorama.jpg")
    args = ap.parse_args()

    imgs = load_images(args.inputs)
    if len(imgs) < 2:
        sys.exit("至少需要 2 张照片")
    pano, ok = stitch(imgs)
    cv2.imwrite(args.out, pano)
    print(f"{'拼接成功' if ok else '已保底横排'} -> {args.out}  "
          f"({pano.shape[1]}x{pano.shape[0]})")


if __name__ == "__main__":
    main()
