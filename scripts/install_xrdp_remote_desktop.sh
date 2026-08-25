#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "Run this script with sudo."
  exit 1
fi

apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y xrdp xorgxrdp

# Allow xrdp to read the system TLS certificate used by the packaged service.
adduser xrdp ssl-cert

systemctl enable xrdp xrdp-sesman
systemctl restart xrdp-sesman xrdp

# Do not expose RDP broadly when UFW is active. ARGOS is on 192.168.0.0/24.
if command -v ufw >/dev/null 2>&1; then
  if ufw status | grep -q '^Status: active'; then
    ufw allow from 192.168.0.0/24 to any port 3389 proto tcp
  fi
fi

echo
echo "XRDP installation complete."
systemctl --no-pager --full status xrdp | sed -n '1,12p'
ss -ltnp | grep ':3389' || true
