#!/usr/bin/env bash
# ============================================================
# elrs_build_flash.sh — Build & Flash ExpressLRS via WiFi
# ============================================================
# Usage:
#   ./elrs_build_flash.sh <DEVICE_IP> [JSON_NUM] [TARGET_NAME]
#
# Examples:
#   ./elrs_build_flash.sh 192.168.3.32              # default: config 20, ESP8285 2400 RX
#   ./elrs_build_flash.sh 192.168.3.32 20           # explicit config number
#   ./elrs_build_flash.sh 192.168.3.32 114 Unified_ESP32_2400_RX_via_WIFI
#
# Common config numbers for ESP8285 2400 RX:
#   16) Generic ESP8285 2.4Ghz RX
#   17) Generic ESP8285 5xPWM 2.4Ghz RX
#   18) Generic ESP8285 6xPWM 2.4Ghz RX
#   19) Generic ESP8285 7xPWM 2.4Ghz RX
#   20) Generic ESP8285 GYRO 2.4Ghz RX   (gyro-elrs.json)
#   21) Generic ESP8285 PA 2.4Ghz RX
# ============================================================

set -euo pipefail

# --- config ---
DEVICE_IP="${1:?Usage: $0 <DEVICE_IP> [JSON_NUM] [TARGET_NAME]}"
JSON_NUM="${2:-20}"
TARGET_NAME="${3:-Unified_ESP8285_2400_RX_via_WIFI}"

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
SRC_DIR="${PROJECT_DIR}/src"
BUILD_DIR="${SRC_DIR}/.pio/build/${TARGET_NAME}"

echo "=============================================="
echo " ELRS Build & Flash"
echo "   Target:  ${TARGET_NAME}"
echo "   Config:  #${JSON_NUM}"
echo "   Device:  ${DEVICE_IP}"
echo "=============================================="

# --- Step 1: Build ---
echo ""
echo "[1/3] Building firmware..."

cd "${SRC_DIR}"

python3 -c "
import pty, os, select, time, sys
target = '${TARGET_NAME}'
choice = '${JSON_NUM}'
pid, fd = pty.fork()
if pid == 0:
    os.execvp('pio', ['pio', 'run', '-e', target])
else:
    sent = False
    while True:
        try:
            r, w, e = select.select([fd], [], [], 1.0)
            if r:
                data = os.read(fd, 1024)
                if not data:
                    break
                sys.stdout.buffer.write(data)
                sys.stdout.flush()
                if not sent and b'Choose a configuration' in data:
                    time.sleep(0.3)
                    os.write(fd, (choice + '\n').encode())
                    sent = True
        except OSError:
            break
    os.waitpid(pid, 0)
"

if [ ! -f "${BUILD_DIR}/firmware.bin" ]; then
    echo "ERROR: Build failed — firmware.bin not found"
    exit 1
fi

echo "   Build done."

# --- Step 2: Upload ---
echo ""
echo "[2/3] Uploading to ${DEVICE_IP}..."

FILESIZE=$(stat -c%s "${BUILD_DIR}/firmware.bin")

RESPONSE=$(curl -s -X POST "http://${DEVICE_IP}/update" \
    -H "X-FileSize: ${FILESIZE}" \
    -F "data=@${BUILD_DIR}/firmware.bin" \
    -F "force=1" \
    --noproxy "*" \
    --max-time 300)

echo "   Response: ${RESPONSE}"

if ! echo "${RESPONSE}" | grep -q '"status": *"ok"'; then
    echo "ERROR: Upload may have failed"
    exit 1
fi

echo "   Upload done."

# --- Step 3: Verify ---
echo ""
echo "[3/3] Waiting for reboot and verifying..."

sleep 12

HW_JSON=$(curl -s --noproxy "*" --max-time 10 "http://${DEVICE_IP}/hardware.json" 2>/dev/null || echo "{}")

if echo "${HW_JSON}" | grep -q '"customised": *true'; then
    echo "   Hardware config loaded OK:"
    echo "${HW_JSON}" | python3 -m json.tool 2>/dev/null || echo "${HW_JSON}"
else
    echo "   WARNING: hardware.json not available or config not customised"
    echo "   ${HW_JSON}"
fi

echo ""
echo "=============================================="
echo " Done! Firmware flashed to ${DEVICE_IP}"
echo "=============================================="
