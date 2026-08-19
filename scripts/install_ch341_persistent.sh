#!/bin/bash
#
# CH341(CH340) USB serial 드라이버 영구 설치 + ARGOS 좌우 포트 고정
#
#   sudo ~/argos_project/scripts/install_ch341_persistent.sh
#
# 배경
# ----
# Jetson 기본 커널에 ch341 모듈이 없어 WCH 드라이버를 직접 빌드했다.
# 그런데 insmod 로만 올려두면 재부팅이나 모듈 언로드 시 사라지고,
# 그러면 /dev/ttyCH341USB* 가 통째로 없어져 로봇이 전혀 동작하지 않는다.
# (2026-08-20 실제로 이 현상 발생)
#
# 또한 CH340 두 개는 VID:PID 가 같고 시리얼 번호가 없어서
# 열거 순서가 바뀌면 좌우가 뒤바뀐다.
# -> 물리 USB 포트 경로로 고정 symlink 를 만든다.

set -e

if [ "$(id -u)" -ne 0 ]; then
    echo "sudo 로 실행해야 합니다."
    exit 1
fi

SRC="/home/odyssey/ch341ser_linux/driver"
KREL="$(uname -r)"
DEST="/lib/modules/${KREL}/kernel/drivers/usb/serial"

echo "=== 1. 모듈 빌드 확인 ==="

if [ ! -f "${SRC}/ch341.ko" ]; then
    echo "ch341.ko 가 없습니다. 먼저 빌드하세요:"
    echo "  cd ${SRC} && make clean && make"
    exit 1
fi

echo "=== 2. 모듈 설치 (${DEST}) ==="

mkdir -p "${DEST}"
cp -f "${SRC}/ch341.ko" "${DEST}/"
depmod -a

echo "=== 3. 부팅 시 자동 적재 ==="

echo "ch341" > /etc/modules-load.d/ch341.conf

echo "=== 4. 지금 적재 ==="

modprobe ch341 2>/dev/null || insmod "${SRC}/ch341.ko" || true

sleep 2

echo "=== 5. udev 규칙 (좌우 포트 고정) ==="

# ARGOS_RIGHT_PATH / ARGOS_LEFT_PATH 는 실제 물리 포트 경로.
# 값이 비어 있으면 규칙을 만들지 않는다.
# 2026-08-20 실측 매핑:
#   /dev/ttyCH341USB0 = 1-2.2 = 오른쪽 UNO (모터 명령 + 오른쪽 엔코더)
#   /dev/ttyCH341USB1 = 1-2.4 = 왼쪽 UNO   (왼쪽 엔코더)
ARGOS_RIGHT_PATH="${ARGOS_RIGHT_PATH:-1-2.2}"
ARGOS_LEFT_PATH="${ARGOS_LEFT_PATH:-1-2.4}"

if [ -n "${ARGOS_RIGHT_PATH}" ] && [ -n "${ARGOS_LEFT_PATH}" ]; then

    cat > /etc/udev/rules.d/99-argos-serial.rules <<RULES
# ARGOS CH340 좌우 고정
# CH340 두 개는 VID:PID 가 같고 serial 이 없으므로 물리 포트 경로로 구분한다.
# 케이블을 다른 USB 포트에 꽂으면 이 규칙을 다시 만들어야 한다.
SUBSYSTEM=="tty", KERNELS=="${ARGOS_RIGHT_PATH}", MODE="0666", GROUP="dialout", SYMLINK+="argos_right"
SUBSYSTEM=="tty", KERNELS=="${ARGOS_LEFT_PATH}",  MODE="0666", GROUP="dialout", SYMLINK+="argos_left"
RULES

    udevadm control --reload-rules
    udevadm trigger --subsystem-match=tty

    sleep 2

    echo "  /dev/argos_right -> $(readlink -f /dev/argos_right 2>/dev/null || echo '(생성 안 됨)')"
    echo "  /dev/argos_left  -> $(readlink -f /dev/argos_left 2>/dev/null || echo '(생성 안 됨)')"

else
    echo "  ARGOS_RIGHT_PATH / ARGOS_LEFT_PATH 미지정 -> udev 규칙 건너뜀"
fi

echo
echo "=== 결과 ==="
lsmod | grep -q ch341 && echo "  모듈 적재됨" || echo "  [경고] 모듈 적재 실패"
ls -l /dev/ttyCH341USB* 2>/dev/null || echo "  [경고] /dev/ttyCH341USB* 없음"
