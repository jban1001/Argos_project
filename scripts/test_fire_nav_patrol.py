#!/usr/bin/env python3

"""fire_nav_patrol 순수 계산 회귀 테스트 (하드웨어 불필요)."""

import importlib.util
import math
import random
from pathlib import Path

import numpy as np


spec = importlib.util.spec_from_file_location(
    "fire_nav_patrol",
    str(Path(__file__).resolve().parent / "fire_nav_patrol.py"),
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_sector_clearance():
    ranges = [float("inf")] * 9
    ranges[4] = 1.2

    result = module.sector_clearance(
        ranges=ranges,
        angle_min=-math.pi,
        angle_increment=math.pi / 4.0,
        range_min=0.1,
        range_max=8.0,
        laser_x=0.1,
        laser_yaw=0.0,
        heading=0.0,
        half_angle=math.radians(20.0),
    )

    assert result is not None
    assert abs(result - 1.1) < 1e-6


def test_select_patrol_target_in_free_space():
    grid = np.zeros((100, 100), dtype=np.int16)
    rng = random.Random(7)

    target = module.select_patrol_target(
        grid=grid,
        resolution=0.05,
        origin_x=-2.5,
        origin_y=-2.5,
        robot_x=0.0,
        robot_y=0.0,
        min_radius=0.8,
        max_radius=1.2,
        clearance=0.3,
        free_max=10,
        attempts=50,
        rng=rng,
    )

    assert target is not None
    x, y, yaw = target
    distance = math.hypot(x, y)
    assert 0.8 <= distance <= 1.2
    assert abs(module.wrap(math.atan2(y, x) - yaw)) < 1e-6


def test_unknown_start_is_rejected():
    grid = np.full((40, 40), -1, dtype=np.int16)

    target = module.select_patrol_target(
        grid=grid,
        resolution=0.05,
        origin_x=-1.0,
        origin_y=-1.0,
        robot_x=0.0,
        robot_y=0.0,
        min_radius=0.5,
        max_radius=0.8,
        clearance=0.2,
        free_max=10,
        attempts=20,
        rng=random.Random(1),
    )

    assert target is None


def test_danger_gates():
    detection = {
        "sensor_ok": True,
        "prob": 0.71,
        "confs": {"fire": 0.21},
    }
    assert module.danger_active(detection, "mlp", 0.70, 0.20)
    assert module.danger_active(detection, "yolo", 0.70, 0.20)

    detection["sensor_ok"] = False
    assert not module.danger_active(detection, "mlp", 0.70, 0.20)
    assert module.danger_active(detection, "yolo", 0.70, 0.20)

    detection["confs"]["fire"] = 0.19
    assert not module.danger_active(detection, "yolo", 0.70, 0.20)


def test_alert_map_overlay():
    grid = np.zeros((100, 120), dtype=np.int16)
    grid[:, 0] = 100
    grid[:, -1] = 100
    grid[0, :] = 100
    grid[-1, :] = 100

    image = module.render_alert_map(
        grid,
        resolution=0.05,
        origin_x=-3.0,
        origin_y=-2.5,
        origin_yaw=0.0,
        robot_x=0.0,
        robot_y=0.0,
        robot_yaw=0.0,
        fire_bearing=math.radians(15.0),
        direction_line_m=1.5,
    )

    assert image is not None
    assert image.ndim == 3 and image.shape[2] == 3
    red_pixels = np.count_nonzero(
        (image[:, :, 2] > 200)
        & (image[:, :, 1] < 80)
        & (image[:, :, 0] < 80)
    )
    orange_pixels = np.count_nonzero(
        (image[:, :, 2] > 200)
        & (image[:, :, 1] > 100)
        & (image[:, :, 1] < 220)
        & (image[:, :, 0] < 80)
    )
    assert red_pixels > 20
    assert orange_pixels > 20


if __name__ == "__main__":
    test_sector_clearance()
    test_select_patrol_target_in_free_space()
    test_unknown_start_is_rejected()
    test_danger_gates()
    test_alert_map_overlay()
    print("fire_nav_patrol tests: OK")
