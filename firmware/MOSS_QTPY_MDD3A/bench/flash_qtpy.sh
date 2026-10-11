#!/bin/bash
# Flash the MOSS V0.4 firmware on the QT Py ESP32-S3 from the onboard computer, through its native USB.
# Step 1: ask the running Arduino app to reboot into the ROM bootloader (DTR/RTS trick);
# step 2: write; step 3: reset out of the bootloader by toggling RTS with the port held open.
#
#   firmware/MOSS_QTPY_MDD3A/bench/flash_qtpy.sh                  images from the default build dir
#   FW_DIR=<dir> firmware/MOSS_QTPY_MDD3A/bench/flash_qtpy.sh     images from <dir>
#
# FW_DIR holds the images `arduino-cli compile ... --output-dir <dir>` writes (see README.md
# here): MOSS_QTPY_MDD3A.ino.bootloader.bin, .partitions.bin, .bin and boot_app0.bin. By default,
# the build dir next to the sketch (gitignored: compile first).
# PY needs pyserial and esptool: the 'firmware' extra of this package, in this checkout's venv
# (uv pip install --python .venv/bin/python -e '.[firmware]', or .venv/bin/pip install -e '.[firmware]').
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
PY="${PY:-$(cd "$HERE/../../.." && pwd)/.venv/bin/python}"   # this checkout's venv; PY=... picks another
FW_DIR="${FW_DIR:-$HERE/../build/esp32.esp32.adafruit_qtpy_esp32s3_nopsram}"
IMAGES="MOSS_QTPY_MDD3A.ino.bootloader.bin MOSS_QTPY_MDD3A.ino.partitions.bin boot_app0.bin MOSS_QTPY_MDD3A.ino.bin"
for f in $IMAGES; do
  [ -f "$FW_DIR/$f" ] || {
    echo "flash_qtpy: no $f in $FW_DIR - compile first (arduino-cli compile ... --output-dir <dir>, see README.md next to this script), then FW_DIR=<dir> $0" >&2
    exit 1; }
done
[ -x "$PY" ] || { echo "flash_qtpy: no python at $PY - make the checkout's .venv (Software.md), or PY=<python with esptool and pyserial>" >&2; exit 1; }
"$PY" -c 'import esptool, serial' 2>/dev/null || {
  echo "flash_qtpy: $PY cannot import esptool and pyserial - install the 'firmware' extra: uv pip install --python .venv/bin/python -e '.[firmware]' (or .venv/bin/pip install -e '.[firmware]'), or PY=<python with them>" >&2
  exit 1; }
cd "$FW_DIR"
port() { ls /dev/serial/by-id/ 2>/dev/null | grep -E "QT_Py|Espressif" | head -1 | sed 's|^|/dev/serial/by-id/|'; }
P=$(port)
[ -n "$P" ] || { echo "flash_qtpy: no QT Py on USB (/dev/serial/by-id/*QT_Py* or *Espressif*) - cable, power" >&2; exit 1; }
if [[ "$P" == *QT_Py* ]]; then
  "$PY" -m esptool --chip esp32s3 --port "$P" --before default-reset --after no-reset chip-id >/dev/null 2>&1 || true
  sleep 3; P=$(port)
fi
echo "bootloader port: $P"
[ -n "$P" ] || { echo "flash_qtpy: the board left USB and did not come back as a bootloader - power-cycle it, then rerun" >&2; exit 1; }
if ! OUT=$("$PY" -m esptool --chip esp32s3 --port "$P" --baud 921600 --before no-reset --after watchdog-reset write-flash -z \
    --flash-mode dio --flash-freq 80m --flash-size 8MB \
    0x0 MOSS_QTPY_MDD3A.ino.bootloader.bin 0x8000 MOSS_QTPY_MDD3A.ino.partitions.bin 0xe000 boot_app0.bin 0x10000 MOSS_QTPY_MDD3A.ino.bin 2>&1); then
  printf '%s\n' "$OUT" | tail -n 15 >&2
  echo "flash_qtpy: the write failed (esptool's last lines above)" >&2
  exit 1
fi
printf '%s\n' "$OUT" | grep -E "Wrote|verified" || true
"$PY" - <<PYEOF
import serial, time
s = serial.Serial("$P", 115200, timeout=0.1)
s.dtr = False; s.rts = True; time.sleep(0.1); s.rts = False
try:
    time.sleep(0.5); s.close()
except Exception: pass
PYEOF
sleep 4
ls /dev/serial/by-id/ 2>/dev/null | grep -E "QT_Py|Espressif" \
  || { echo "flash_qtpy: the board is not back on USB - power-cycle it" >&2; exit 1; }
