#!/usr/bin/env python3

"""
저장된 map(pgm)을 보기 좋은 PNG 로 변환한다.

    python3 map_to_png.py maps/argos_lab_imu_v2.yaml
    python3 map_to_png.py maps/argos_lab_imu_v2.yaml --scale 6
    python3 map_to_png.py maps/a.yaml maps/b.yaml --compare 비교.png

왜 그냥 pgm 을 안 쓰는가
------------------------
nav2 가 저장하는 map 은 146x84 처럼 아주 작다. 이미지 뷰어가 기본으로
부드럽게 늘려서 보여주기 때문에 벽이 흐릿해지고 이중선이 뭉개져
지도 품질을 눈으로 비교할 수 없다.
여기서는 nearest neighbor 로 확대해 격자를 그대로 살린다.

ROS map 규약 (trinary)
  254 흰색 = 자유 공간
  205 회색 = 미탐색
    0 검정 = 장애물
"""

from __future__ import annotations

import argparse
import os

import numpy as np
import yaml
from PIL import Image


FREE = 254
UNKNOWN = 205


def load_map(yaml_path: str):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)

    pgm = meta["image"]
    if not os.path.isabs(pgm):
        pgm = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), pgm)

    img = Image.open(pgm)
    return np.array(img), meta


def colorize(arr: np.ndarray) -> Image.Image:
    """미탐색을 옅은 파랑으로 칠해 자유공간과 확실히 구분한다."""

    h, w = arr.shape
    rgb = np.zeros((h, w, 3), dtype=np.uint8)

    unknown = np.abs(arr.astype(np.int16) - UNKNOWN) <= 2
    free = arr >= 250
    occ = ~unknown & ~free

    rgb[free] = (255, 255, 255)
    rgb[unknown] = (216, 226, 238)

    # 장애물은 원래 밝기를 유지해 확률 차이를 남긴다
    v = arr[occ].astype(np.uint8)
    rgb[occ] = np.stack([v, v, v], axis=-1)

    return Image.fromarray(rgb, mode="RGB")


def upscale(img: Image.Image, scale: int) -> Image.Image:
    if scale <= 1:
        return img
    return img.resize(
        (img.width * scale, img.height * scale), Image.NEAREST
    )


def stats(arr: np.ndarray, meta: dict) -> str:
    unknown = np.abs(arr.astype(np.int16) - UNKNOWN) <= 2
    free = arr >= 250
    occ = ~unknown & ~free

    res = float(meta.get("resolution", 0.05))
    cell = res * res

    return (
        "  크기 {}x{} px, 해상도 {:.3f} m/px  ({:.1f} x {:.1f} m)\n"
        "  자유 {:6d} px = {:6.2f} m^2\n"
        "  장애물 {:4d} px = {:6.2f} m^2\n"
        "  미탐색 {:4d} px"
    ).format(
        arr.shape[1], arr.shape[0], res,
        arr.shape[1] * res, arr.shape[0] * res,
        int(free.sum()), float(free.sum()) * cell,
        int(occ.sum()), float(occ.sum()) * cell,
        int(unknown.sum()),
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("yaml", nargs="+", help="map yaml 경로")
    p.add_argument("--scale", type=int, default=6,
                   help="nearest neighbor 확대 배율 (기본 6)")
    p.add_argument("--compare", default=None,
                   help="두 개 이상 줬을 때 가로로 이어붙일 PNG 경로")
    args = p.parse_args()

    imgs = []

    for y in args.yaml:
        arr, meta = load_map(y)
        img = upscale(colorize(arr), args.scale)

        out = os.path.splitext(os.path.abspath(y))[0] + ".png"
        img.save(out)

        print("{}\n{}\n  -> {}\n".format(
            os.path.basename(y), stats(arr, meta), out))

        imgs.append((os.path.basename(y), img))

    if args.compare and len(imgs) >= 2:
        pad = 16
        w = sum(i.width for _, i in imgs) + pad * (len(imgs) + 1)
        h = max(i.height for _, i in imgs) + pad * 2

        canvas = Image.new("RGB", (w, h), (245, 245, 245))

        x = pad
        for _, i in imgs:
            canvas.paste(i, (x, pad))
            x += i.width + pad

        canvas.save(args.compare)
        print("비교 이미지 -> {}".format(args.compare))


if __name__ == "__main__":
    main()
