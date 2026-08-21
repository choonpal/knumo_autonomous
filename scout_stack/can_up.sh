#!/bin/bash
# Scout Mini CAN 어댑터(gs_usb)를 can0 이름으로 올린다.
#
# 젯슨 내장 CAN(mttcan)이 부팅 때 can0을 선점하므로, 그것을 canhw0으로 밀어내고
# USB 어댑터를 can0으로 개명한다. scout_mini_description/urdf/scout_mini.urdf.xacro
# 가 interface=can0 을 하드코딩하고 있어서 이름이 can0이어야 한다.
#
# 재부팅하면 원상복귀되므로 부팅 후 매번 실행:  sudo bash ~/can_up.sh

if ! lsmod | grep -q '^gs_usb'; then
    echo "[*] gs_usb 모듈 로드"
    insmod /home/knumo/gs_usb.ko || exit 1
    sleep 1
fi

if [ ! -e /sys/class/net/can1 ]; then
    echo "[!] can1(gs_usb 어댑터)이 없다. USB CAN 어댑터가 꽂혀 있는지 확인할 것."
    ip -br link | grep -i can
    exit 1
fi

echo "[*] can0(내장 mttcan) -> canhw0, can1(어댑터) -> can0"
ip link set can0 down
ip link set can1 down
ip link set can0 name canhw0
ip link set can1 name can0
ip link set can0 up type can bitrate 500000

echo "--- 결과 ---"
ip -br link | grep -i can
echo "--- 프레임 확인 (3초) ---"
timeout 3 candump -n 5 can0 || echo "프레임 없음: Scout 전원/배선 확인"
