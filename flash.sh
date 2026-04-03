#!/bin/bash
set -e

ARDUINO_CLI='/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli'
FQBN="esp32:esp32:lilygo_t_display"
SKETCH_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="/tmp/build_dashboard_display"
PORT="${1:-/dev/cu.wchusbserial59680472421}"

echo "── Dashboard Display firmware ──"
echo "Board:  $FQBN"
echo "Port:   $PORT"
echo "Sketch: $SKETCH_DIR"
echo ""

echo "▸ Compiling..."
"$ARDUINO_CLI" compile --fqbn "$FQBN" \
  --build-property "build.partitions=min_spiffs" \
  --build-property "upload.maximum_size=1966080" \
  --build-path "$BUILD_DIR" \
  "$SKETCH_DIR"

echo ""
echo "▸ Uploading..."
"$ARDUINO_CLI" upload --fqbn "$FQBN" --port "$PORT" \
  --input-dir "$BUILD_DIR" \
  "$SKETCH_DIR"

echo ""
echo "✔ Done"
