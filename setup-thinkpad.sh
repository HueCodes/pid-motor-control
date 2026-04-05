#!/usr/bin/env bash
# pid-motor-control: ThinkPad setup and build
# Prerequisites: arm-none-eabi-gcc, cmake, st-flash (from thinkpad-setup.sh)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CUBE_DIR="$HOME/STM32Cube/Repository/STM32Cube_FW_F4"

echo "=== pid-motor-control setup ==="

# Check toolchain
for tool in arm-none-eabi-gcc cmake st-flash; do
    if ! command -v "$tool" &>/dev/null; then
        echo "ERROR: $tool not found. Run ../thinkpad-setup.sh first."
        exit 1
    fi
done

# Check STM32CubeF4 (needed for CMSIS headers)
if [ ! -d "$CUBE_DIR" ]; then
    echo "ERROR: STM32CubeF4 not found at $CUBE_DIR"
    echo "Run ../thinkpad-setup.sh or:"
    echo "  git clone --depth 1 -b v1.28.1 https://github.com/STMicroelectronics/STM32CubeF4.git $CUBE_DIR"
    exit 1
fi

# Build
echo "Building..."
cd "$SCRIPT_DIR"
rm -rf build
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j"$(nproc)"

echo ""
echo "Build complete. Output:"
ls -lh build/pid-motor-control.bin build/pid-motor-control.hex

echo ""
echo "To flash (plug in Nucleo-F411RE via USB):"
echo "  cd $SCRIPT_DIR/build"
echo "  st-flash write pid-motor-control.bin 0x08000000"
echo ""
echo "Or: cmake --build build --target flash"

echo ""
echo "=== Wiring ==="
echo "  TB6612FNG motor driver:"
echo "    PWMA  -> PA5 (TIM2_CH1 or TIM3)"
echo "    AIN1  -> PA6"
echo "    AIN2  -> PA8"
echo "    STBY  -> PA9"
echo "    VM    -> Motor supply (6-12V)"
echo "    VCC   -> 3.3V"
echo "    GND   -> GND"
echo ""
echo "  MPU-6050 IMU (I2C1):"
echo "    SDA   -> PB9"
echo "    SCL   -> PB8"
echo "    VCC   -> 3.3V"
echo "    GND   -> GND"
echo ""
echo "  SSD1306 OLED (I2C1, shared bus):"
echo "    SDA   -> PB9"
echo "    SCL   -> PB8"
echo "    VCC   -> 3.3V"
echo "    GND   -> GND"
echo ""
echo "  DC Motor -> TB6612FNG AOUT1/AOUT2"
echo "  USART2 (PA2/PA3) -> ST-Link VCP (built into Nucleo, just USB)"
echo ""
echo "Serial monitor: screen /dev/ttyACM0 115200"
