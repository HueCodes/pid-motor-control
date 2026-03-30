# PID Motor Control

Closed-loop PID motor controller on STM32 Nucleo-F411RE with real-time IMU feedback.

## Hardware

| Component | Part | Interface |
|-----------|------|-----------|
| MCU | STM32 Nucleo-F411RE | - |
| Motor Driver | TB6612FNG | GPIO + PWM |
| IMU | MPU-6050 (6-axis) | I2C1 @ 400 kHz |
| Display | SSD1306 0.96" OLED | I2C1 @ 0x3C |
| Motor | 130-type DC | via TB6612FNG |
| Debug | ST-Link VCP (USART2) | 115200 baud |

## Wiring

```
STM32 Nucleo-F411RE          TB6612FNG
  PA6 (TIM3_CH1) ----------> PWMA
  PA5 ----------------------> AIN1
  PA8 ----------------------> AIN2
  PA9 ----------------------> STBY
  3.3V ---------------------> VCC
  GND ---------------------> GND
  (ext 5-12V) -------------> VM

STM32                         MPU-6050
  PB8 (I2C1_SCL) --+-------> SCL
  PB9 (I2C1_SDA) --+-------> SDA
  3.3V -------[4.7K]--+      VCC <--- 3.3V
              [4.7K]--+      GND <--- GND
  (pull-ups to 3.3V)         AD0 <--- GND (addr 0x68)

STM32                         SSD1306 OLED
  PB8 (shared SCL) ---------> SCL
  PB9 (shared SDA) ---------> SDA
  3.3V ---------------------> VCC
  GND ---------------------> GND

STM32                         DC Motor
  (motor connects to TB6612FNG AO1/AO2 outputs)

USART2 (PA2/PA3) is routed to ST-Link USB -- no extra wiring needed.
```

## Architecture

```
TIM3_CH1 (20 kHz PWM) --> TB6612FNG --> DC Motor
                                          |
MPU-6050 (I2C, 14-byte burst read) <-----+
    |                                  (physical coupling)
    v
Complementary Filter (alpha=0.98)
    |
    v
PID Controller (1 kHz, TIM2 ISR)
    |  - Derivative on measurement
    |  - Low-pass filter on D term (N=10)
    |  - Integral clamping anti-windup
    v
Motor effort (-1.0 to +1.0) --> sign-magnitude drive

Main loop (background):
  - UART CLI: tune Kp/Ki/Kd, trigger step response
  - OLED: live display of gains, setpoint, error, output bar
  - Step response: CSV over UART, plot with scripts/plot_step_response.py
```

## Build

Requires `arm-none-eabi-gcc` and CMake.

```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
```

Flash via ST-Link:
```bash
make flash
# or: st-flash write pid-motor-control.bin 0x08000000
```

## Usage

Connect serial terminal at 115200 baud:
```bash
screen /dev/tty.usbmodem* 115200
```

Commands:
```
kp <val>    Set proportional gain
ki <val>    Set integral gain
kd <val>    Set derivative gain
sp <val>    Set setpoint angle (degrees)
step <val>  Step to angle + log 2s of CSV data
reset       Zero PID integrator
status      Print current state
help        Command list
```

Capture and plot step response:
```bash
# In another terminal, capture serial output:
# Then in the serial console: step 30
python3 scripts/plot_step_response.py log.csv
```

## Tuning Procedure

1. Start with `kp 1.0`, `ki 0`, `kd 0`
2. Increase Kp until the system oscillates, then back off 20-30%
3. Add Ki from a small value until steady-state error is eliminated
4. Add Kd to dampen oscillations
5. Use `step <angle>` to capture response data for quantitative analysis

## Milestones

- [x] PWM output driving motor at variable duty cycle
- [x] I2C reads from MPU-6050 (raw accel + gyro)
- [x] PID loop with proportional-only control
- [x] Full PID with integral anti-windup and derivative filtering
- [x] UART CLI for live tuning
- [x] Step response capture and export
- [x] OLED real-time display

## Key Concepts Demonstrated

- Bare-metal timer configuration (PWM mode, output compare, preload registers)
- I2C master driver with multi-byte burst reads at 400 kHz
- Fixed-rate control loop via timer interrupt (1 kHz)
- Complementary filter for IMU sensor fusion
- PID with derivative-on-measurement and anti-windup
- Ring-buffer UART with interrupt-driven RX
- Real-time parameter tuning and step response analysis

## Pin Summary

| Pin | Function | Peripheral |
|-----|----------|------------|
| PA2 | USART2 TX | ST-Link VCP |
| PA3 | USART2 RX | ST-Link VCP |
| PA5 | AIN1 | Motor direction |
| PA6 | TIM3_CH1 | Motor PWM (20 kHz) |
| PA8 | AIN2 | Motor direction |
| PA9 | STBY | Motor enable |
| PB8 | I2C1_SCL | MPU-6050 + OLED |
| PB9 | I2C1_SDA | MPU-6050 + OLED |
