#!/usr/bin/env python3

"""
YOLO 가 실제 불꽃을 몇 점으로 보는지 측정한다.

모터를 전혀 쓰지 않는다. 로봇은 움직이지 않는다.
라이터/촛불을 카메라 앞에서 켜고 숫자를 보면 된다.

왜 필요한가
-----------
fire_seeker 의 --gate yolo 는 fire confidence 가
YOLO_ONLY_FIRE_CONF(기본 0.40) 이상일 때만 불로 판단한다.
라이터 불꽃은 작아서 이 기준을 못 넘을 수 있다.
실제로 2026-08-25 주행에서 라이터를 켰는데도 화재 상태 진입이 0회였다.

여기서 나온 최고값을 보고 --fire-conf 를 정하면 된다.
    최고값이 0.25 였다면  --fire-conf 0.15  정도

방위각 부호도 같이 확인한다.
불을 로봇 기준 왼쪽에 두면 방위각이 + 로 나와야 한다.
반대로 나오면 카메라가 뒤를 보고 있거나 좌우가 뒤집힌 것이므로
fire_seeker 의 CAMERA_YAW_OFFSET 을 고쳐야 한다.

사용
----
    source /opt/ros/jazzy/setup.bash
    ~/.venv/bin/python ~/argos_project/scripts/check_fire_detect.py [초]
"""

import sys
import math
import time
import importlib.util
from pathlib import Path

import cv2


SCRIPT_DIR = Path(__file__).resolve().parent


def load_fire_seeker():

    spec = importlib.util.spec_from_file_location(
        "fs", str(SCRIPT_DIR / "fire_seeker.py")
    )

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    return module


def main():

    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0

    fs = load_fire_seeker()

    # 아두이노/MLP 없이 YOLO 만 본다
    fs.REQUIRE_SENSOR_GATE = False

    det = fs.FireDetector()

    print()
    print("=" * 62)
    print(f" 카메라 앞에서 불을 켜 주세요. {duration:.0f}초 측정합니다.")
    print(" 로봇은 움직이지 않습니다.")
    print("=" * 62)
    print(f"{'t':>5} {'fire':>6} {'smoke':>6} {'spark':>6} {'cig':>6} "
          f"{'방위':>9}")
    print("-" * 62)

    best = {"conf": -1.0, "frame": None, "t": 0.0}

    peak_fire = 0.0
    peak_any = 0.0
    hits_10 = 0
    hits_40 = 0
    frames = 0

    t0 = time.time()

    try:
        while time.time() - t0 < duration:

            d = det.step()

            if not d.get("ok"):
                time.sleep(0.05)
                continue

            frames += 1

            c = d["confs"]
            f = c.get("fire", 0.0)

            any_c = max(
                c.get("fire", 0.0),
                c.get("smoke", 0.0),
                c.get("spark", 0.0),
            )

            peak_fire = max(peak_fire, f)
            peak_any = max(peak_any, any_c)

            if f >= 0.10:
                hits_10 += 1
            if f >= 0.40:
                hits_40 += 1

            if any_c > best["conf"]:
                best.update(
                    conf=any_c,
                    frame=d.get("frame"),
                    t=time.time() - t0,
                )

            b = d["bearing"]
            b_txt = "없음" if b is None else f"{math.degrees(b):+.0f}deg"

            # 조용할 때는 안 찍는다
            if f >= 0.05 or any_c >= 0.20:
                print(
                    f"{time.time() - t0:5.1f} {f:6.3f} "
                    f"{c.get('smoke', 0.0):6.3f} "
                    f"{c.get('spark', 0.0):6.3f} "
                    f"{c.get('cigarette_butt', 0.0):6.3f} "
                    f"{b_txt:>9}",
                    flush=True,
                )

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[중단]")

    finally:
        print()
        print("=" * 62)
        print(f" 처리 프레임            : {frames}")
        print(f" fire conf 최고값       : {peak_fire:.3f}")
        print(f" fire/smoke/spark 최고  : {peak_any:.3f}")
        print(f" fire >= 0.10 프레임수  : {hits_10}")
        print(f" fire >= 0.40 프레임수  : {hits_40}  (기본 기준)")
        print()

        if peak_fire < 0.05:
            print(" 판정: YOLO 가 불꽃을 거의 못 봤다.")
            print("       카메라가 불을 향하고 있는지, 거리가 너무 멀지 않은지,")
            print("       /dev/video0 이 맞는 카메라인지 확인할 것.")
        elif hits_40 == 0:
            rec = max(0.05, round(peak_fire * 0.6, 2))
            print(f" 판정: 불은 보이는데 기본 기준 0.40 을 못 넘었다.")
            print(f"       --fire-conf {rec} 정도로 낮춰서 쓸 것.")
        else:
            print(" 판정: 기본 기준 0.40 으로도 잡힌다.")

        if best["frame"] is not None:
            p = str(SCRIPT_DIR.parent / "maps" / "fire_detect_best.jpg")
            cv2.imwrite(p, best["frame"])
            print(f"\n 최고 프레임 저장: {p}")
            print(f"   t={best['t']:.1f}s  conf={best['conf']:.3f}")

        det.release()


if __name__ == "__main__":
    main()
