#!/usr/bin/env bash
# Turn compass Wi‑Fi on via N2K magic PGN, OTA-flash firmware, then Wi‑Fi off.
#
# Prerequisites:
#   - Device already running firmware that handles PGN 61184 WIFI
#   - SocketCAN on the same NMEA 2000 bus (default can0)
#   - wifi_secrets.h with OTA_PASSWORD (and Wi‑Fi credentials on the device)
#
# Usage:
#   ./ota_update.sh                     # build + OTA to wit-compass.local
#   ./ota_update.sh 192.168.1.139       # explicit host/IP
#   HOST=192.168.1.139 CAN=can0 ./ota_update.sh
#   ./ota_update.sh --skip-build        # use existing .pio/build/.../firmware.bin
#   ./ota_update.sh --minutes 20        # keep Wi‑Fi window 20 min for the ON PGN

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

HOST="${HOST:-wit-compass.local}"
CAN="${CAN:-can0}"
ENV_NAME="${PIO_ENV:-shesp32}"
MINUTES="${MINUTES:-15}"
SKIP_BUILD=0
# Default leave Wi‑Fi on: magic OFF used to WiFi.mode(WIFI_OFF) and killed N2K.
LEAVE_WIFI_ON=1

usage() {
  sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'
  exit "${1:-0}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage 0 ;;
    --skip-build) SKIP_BUILD=1; shift ;;
    --leave-wifi-on) LEAVE_WIFI_ON=1; shift ;;
    --minutes) MINUTES="$2"; shift 2 ;;
    --can|-i) CAN="$2"; shift 2 ;;
    --host) HOST="$2"; shift 2 ;;
    --env) ENV_NAME="$2"; shift 2 ;;
    --) shift; break ;;
    -*)
      echo "Unknown option: $1" >&2
      usage 2
      ;;
    *)
      HOST="$1"
      shift
      ;;
  esac
done

WIFI_CTL=(python3 "$ROOT/tools/n2k_wifi_control.py" --iface "$CAN")
FIRMWARE="$ROOT/.pio/build/${ENV_NAME}/firmware.bin"

ota_password() {
  local f="$ROOT/wifi_secrets.h"
  if [[ ! -f "$f" ]]; then
    echo "wifi_secrets.h missing — copy from wifi_secrets.h.example" >&2
    exit 1
  fi
  local line
  line="$(grep -E '^[[:space:]]*#define[[:space:]]+OTA_PASSWORD[[:space:]]+' "$f" | head -1 || true)"
  if [[ -z "$line" ]]; then
    echo "OTA_PASSWORD not found in wifi_secrets.h" >&2
    exit 1
  fi
  # #define OTA_PASSWORD "secret"
  sed -n 's/.*OTA_PASSWORD[[:space:]]*"\([^"]*\)".*/\1/p' <<<"$line"
}

find_espota() {
  if [[ -n "${ESPOTA:-}" && -x "$ESPOTA" ]]; then
    echo "$ESPOTA"
    return
  fi
  local candidate
  candidate="$(find "${HOME}/.platformio/packages" -path '*/tools/espota.py' 2>/dev/null | head -1 || true)"
  if [[ -n "$candidate" ]]; then
    echo "$candidate"
    return
  fi
  echo "espota.py not found under ~/.platformio/packages" >&2
  exit 1
}

wait_for_host() {
  local host="$1"
  local seconds="${2:-120}"
  local i
  echo "Waiting up to ${seconds}s for ${host} …"
  for ((i = 0; i < seconds; i++)); do
    if ping -c 1 -W 1 "$host" >/dev/null 2>&1; then
      echo "Host ${host} is up."
      return 0
    fi
    # mDNS / some hosts ignore ICMP; try TCP 3232 (OTA) as well.
    if timeout 1 bash -c "echo >/dev/tcp/${host}/3232" 2>/dev/null; then
      echo "Host ${host}:3232 is open."
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for ${host}" >&2
  return 1
}

AUTH="$(ota_password)"
ESPOTA="$(find_espota)"

echo "==> Wi‑Fi ON via N2K (${CAN}, ${MINUTES} min window)"
"${WIFI_CTL[@]}" on --minutes "$MINUTES"

cleanup() {
  if [[ "$LEAVE_WIFI_ON" -eq 0 ]]; then
    echo "==> Wi‑Fi OFF via N2K (${CAN})"
    "${WIFI_CTL[@]}" off || true
  else
    echo "==> Leaving Wi‑Fi on (--leave-wifi-on)"
  fi
}
trap cleanup EXIT

wait_for_host "$HOST" 120

if [[ "$SKIP_BUILD" -eq 0 ]]; then
  echo "==> Building ${ENV_NAME}"
  if [[ -x "$ROOT/pio" ]]; then
    "$ROOT/pio" run -e "$ENV_NAME"
  else
    pio run -e "$ENV_NAME"
  fi
fi

if [[ ! -f "$FIRMWARE" ]]; then
  echo "Firmware not found: $FIRMWARE" >&2
  exit 1
fi

echo "==> OTA to ${HOST} ($FIRMWARE)"
python3 "$ESPOTA" -i "$HOST" -p 3232 --auth="$AUTH" -f "$FIRMWARE"

echo "==> OTA done."
# trap turns Wi‑Fi off
