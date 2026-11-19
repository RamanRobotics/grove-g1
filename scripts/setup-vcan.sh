#!/usr/bin/env bash
# Creates vcan0, the virtual CAN bus the simulated LinkerBot O6 hands answer on. Run on the host,
# once per boot, before a hand:=linkerbot_o6 simulation:
#
#   ./scripts/setup-vcan.sh            # vcan0
#   ./scripts/setup-vcan.sh vcan1      # another name
#
# On the host because the container shares its network namespace (network_mode: host) but not
# its kernel modules, and creating an interface needs root there. Virtual, so nothing written to
# it reaches a robot: the real hands are on can0, brought up with
#   sudo ip link set can0 up type can bitrate 1000000
set -euo pipefail

IFACE=${1:-vcan0}

if ip link show "$IFACE" >/dev/null 2>&1; then
    echo "$IFACE exists already."
    exit 0
fi

sudo modprobe vcan
sudo ip link add dev "$IFACE" type vcan
sudo ip link set up "$IFACE"
echo "$IFACE is up."
