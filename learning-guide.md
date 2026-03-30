# PID Motor Control -- Learning Guide

## Core Concepts

### PID Control Theory

A PID controller continuously calculates an error value as the difference between a desired setpoint and a measured process variable. The controller output is a weighted sum of three terms: the proportional term (P), which responds to the current error; the integral term (I), which responds to the accumulated error over time; and the derivative term (D), which responds to the rate of change of the error. The general continuous-time equation is `u(t) = Kp * e(t) + Ki * integral(e(t) dt) + Kd * de(t)/dt`, where `e(t) = setpoint - measurement`.

The proportional term produces an output proportional to the current error. A large Kp drives the system aggressively toward the setpoint but introduces overshoot and oscillation. The integral term accumulates past errors, eliminating steady-state offset that proportional-only control cannot remove. However, the integral term is slow to respond and can cause the system to overshoot if the accumulated error grows too large (windup). The derivative term predicts future error by examining the rate of change, providing a damping effect that reduces overshoot and improves stability.

In practice, the three gains (Kp, Ki, Kd) interact with each other and with the plant dynamics. Increasing Kp alone speeds up response but degrades stability margin. Adding Ki removes steady-state error but tends to increase overshoot and settling time. Adding Kd improves transient response and stability but amplifies high-frequency noise. The art of PID tuning is finding the gain combination that meets all performance specifications simultaneously for a given plant.

The error signal `e(t)` is the foundation of everything. If your sensor is noisy, your error signal is noisy, and the derivative term will amplify that noise catastrophically. If your sensor has a bias, the integral term will eventually compensate, but it will take time proportional to the bias magnitude divided by Ki. Understanding the quality of your error signal is the first step before tuning any gains.

### Tuning Methods

**Ziegler-Nichols ultimate gain method**: Set Ki and Kd to zero. Increase Kp until the system oscillates with a sustained, constant-amplitude oscillation. Record this critical gain (Ku) and the oscillation period (Tu). The Ziegler-Nichols table then gives starting values: for a PID controller, Kp = 0.6 * Ku, Ki = 2 * Kp / Tu, Kd = Kp * Tu / 8. These values produce an aggressive response with roughly 25% overshoot, which you then refine manually. This method requires the system to be brought to marginal stability, which may be unacceptable for fragile mechanical systems.

**Manual tuning** follows a systematic approach: start with Kp only (Ki = 0, Kd = 0). Increase Kp until the system responds quickly but begins to oscillate. Back off Kp by 20-30%. Then add Ki starting from a small value, increasing until steady-state error is eliminated without excessive overshoot. Finally, add Kd to dampen oscillations and improve settling time. At each stage, apply a step input and observe the response. This is the method you will use in this project because it builds intuition and your motor system has low inertia, making the Ziegler-Nichols oscillation test somewhat impractical.

**Software-assisted tuning** uses tools like MATLAB's PID Tuner, Python's control library, or frequency-response methods (Bode plots) to design gains analytically. You model the plant transfer function, then use root locus or frequency-domain techniques to place closed-loop poles where desired. For this project, you can capture step response data over UART, import it into Python, fit a first-order-plus-dead-time (FOPDT) model, and compute initial gains using the Cohen-Coon or SIMC methods. This hybrid approach (empirical data plus analytical design) is common in industry.

### Anti-Windup Strategies

Integral windup occurs when the controller output saturates (e.g., PWM maxes out at 100% duty) but the integral term continues accumulating error. When the error eventually changes sign, the oversized integral term must unwind before the output can respond, causing massive overshoot and prolonged settling. This is a critical failure mode in any real control system with actuator saturation, which is every real control system.

**Integral clamping** (also called conditional integration) is the simplest approach: stop integrating when the output is saturated. Implementation: after computing the PID output, check if it exceeds the actuator limits. If it does, clamp the output to the limit and do not update the integral accumulator on that iteration. A refinement is to only freeze the integrator when the error and the output have the same sign (meaning the integrator is pushing further into saturation). This is what you should implement first.

**Back-calculation** is more sophisticated: when the output saturates, the difference between the saturated output and the unsaturated output is fed back through a gain (1/Tt, where Tt is the tracking time constant) to drive the integrator toward a value consistent with the saturated output. This provides a smooth transition out of saturation. The tracking time constant Tt is typically set to `sqrt(Ti * Td)` where Ti = Kp/Ki and Td = Kd/Kp. Back-calculation is preferred in industrial controllers because it handles both windup and bumpless transfer between manual and automatic modes.

For this project, implement clamping first because it is straightforward and effective for a DC motor application. If you observe slow recovery after hitting PWM limits (e.g., when the motor stalls against an obstacle and then releases), revisit with back-calculation.

### Derivative Filtering and Derivative Kick

The pure derivative term `Kd * de/dt` has two practical problems. First, any noise on the measurement signal is amplified. If the gyroscope reading has 0.1 degree/s of noise and you sample at 1 kHz, the noise on the discrete derivative can be enormous. Second, a step change in the setpoint produces a theoretically infinite derivative (derivative kick), causing a spike in the controller output that can damage actuators or produce audible/mechanical transients.

**Derivative filtering** applies a first-order low-pass filter to the derivative term. The filtered derivative transfer function becomes `Kd * s / (1 + s * Kd / (N * Kp))`, where N is the filter coefficient (typically 8 to 20). In discrete time, this is implemented as an exponential moving average on the derivative: `D_filtered[k] = alpha * D_filtered[k-1] + (1 - alpha) * D_raw[k]`, where `alpha = Td / (Td + N * Ts)` and Ts is the sample period. A higher N gives less filtering; a lower N gives more. Start with N = 10.

**Derivative kick** is solved by computing the derivative on the measurement (process variable) rather than the error. Instead of `Kd * d(setpoint - measurement)/dt`, use `Kd * d(-measurement)/dt`. Since the measurement does not jump instantaneously when the setpoint changes, the derivative term remains smooth. This is sometimes called "derivative on measurement" and is the standard practice in all industrial PID implementations. You should implement this from the start; there is no reason to ever differentiate the error signal directly.

Combining both techniques: differentiate the measurement (not the error), then apply a first-order filter. This gives you a well-behaved derivative term that damps oscillations without amplifying noise or producing actuator transients.

### PWM Generation for Motor Control

Pulse Width Modulation controls the average voltage delivered to the motor by rapidly switching the supply on and off. The duty cycle (fraction of time the signal is high) determines the average voltage: 50% duty on a 12V supply delivers an average of 6V. The motor's inductance acts as a natural low-pass filter, smoothing the pulsed current into a roughly constant current, so the motor sees an approximately steady torque proportional to the duty cycle.

**Frequency selection** is critical. Too low (below 1 kHz) and the motor audibly buzzes, the current ripple is large, and the motor heats excessively from I2R losses in the ripple current. Too high (above 50 kHz) and switching losses in the driver transistors increase, the driver may not fully turn on/off, and EMI worsens. For small DC motors with the TB6612FNG, 10-25 kHz is ideal. 20 kHz is the conventional choice because it is above the audible range and well within the TB6612FNG's switching capability (up to 100 kHz per datasheet).

On the STM32F411RE, PWM is generated using timer peripherals in output compare mode. The timer counts from 0 to an auto-reload value (ARR), and the output pin goes high (or low) when the counter matches the capture/compare register (CCR). The duty cycle is CCR/ARR. With a timer clock of 100 MHz and ARR = 4999, the PWM frequency is 100 MHz / 5000 = 20 kHz, and you get 5000 steps of duty cycle resolution (about 12 bits of effective resolution). This is more than adequate for motor control.

To change motor speed in your PID loop, you write the new duty cycle to the CCR register. This takes effect at the next timer update event (at the end of the current PWM period), providing glitch-free updates. The complementary output and dead-time features of TIM1 are useful for full H-bridge driving but are unnecessary with the TB6612FNG since it handles the H-bridge internally.

### H-Bridge Motor Drivers (TB6612FNG)

An H-bridge is a circuit of four switches arranged so that a motor connected across the bridge can be driven in either direction. Closing the top-left and bottom-right switches drives current one way; closing the top-right and bottom-left drives it the other way. Closing both top switches or both bottom switches creates a short circuit (shoot-through), which must be prevented. The TB6612FNG integrates dual H-bridges with MOSFET switches, handling all the shoot-through protection, level shifting, and gate driving internally.

The TB6612FNG has three control inputs per channel: AIN1, AIN2, and PWMA. The truth table is straightforward: AIN1=HIGH, AIN2=LOW drives forward; AIN1=LOW, AIN2=HIGH drives reverse; AIN1=AIN2=LOW is coast (motor floats); AIN1=AIN2=HIGH is brake (motor terminals shorted, providing dynamic braking). The PWM input gates the output, so you connect the STM32's timer PWM output to PWMA and use two GPIO pins for AIN1/AIN2. To change direction in your PID controller, you toggle AIN1/AIN2 and keep the PWM duty cycle as the absolute value of the control effort.

The TB6612FNG handles up to 1.2A continuous (3.2A peak) per channel at 2.5-13.5V motor supply. Your 130-type DC motor typically draws 150-300 mA under load, well within limits. The STBY (standby) pin must be pulled HIGH for operation; tie it to VCC through a 10K resistor or drive it from a GPIO if you want a hardware emergency stop. The VM (motor supply) and VCC (logic supply, 2.7-5.5V) must be decoupled with 100nF ceramic capacitors placed as close to the IC pins as possible. The logic supply VCC can be connected to the Nucleo's 3.3V output since the TB6612FNG accepts 3.3V logic levels.

**Braking modes matter for control.** When your PID controller outputs zero effort, you want the motor to stop quickly, not coast. Use brake mode (AIN1=AIN2=HIGH, PWM=HIGH) for zero output. For non-zero output, use PWM on the PWMA pin with the direction set by AIN1/AIN2. Some implementations use "sign-magnitude" drive (direction pins set, PWM controls magnitude) and others use "locked-antiphase" drive (single PWM between AIN1 and AIN2, 50% = stopped). Sign-magnitude is simpler and is what you should use with the TB6612FNG.

### I2C Protocol

I2C (Inter-Integrated Circuit) is a synchronous, multi-master, multi-slave serial bus using two wires: SDA (data) and SCL (clock). Both lines are open-drain, meaning devices can only pull them low; pull-up resistors (typically 2.2K-4.7K to VCC) pull them high when released. Communication starts with a START condition (SDA falls while SCL is high) and ends with a STOP condition (SDA rises while SCL is high). Between these, data is transmitted in 8-bit bytes, MSB first, with an ACK/NACK bit after each byte.

The master initiates all transactions. A write transaction is: START, 7-bit slave address + write bit (0), wait for ACK, send register address byte, wait for ACK, send data byte(s), each followed by ACK, then STOP. A read transaction is: START, slave address + write bit, ACK, register address, ACK, REPEATED START, slave address + read bit (1), ACK, read data byte(s) (master ACKs all but the last, which gets NACK), then STOP. The repeated start is critical: it allows the master to switch from write (to set the register pointer) to read without releasing the bus.

**Clock stretching** is when the slave holds SCL low to pause the master. The MPU-6050 uses clock stretching, so your I2C master must support it. On the STM32F411RE, the I2C peripheral handles this automatically in hardware. The standard I2C speed is 100 kHz; fast mode is 400 kHz. The MPU-6050 supports 400 kHz, and you should use it because you need to read 14 bytes (accel + temp + gyro) every loop iteration, and at 100 kHz that takes about 1.5 ms (too slow for a 1 kHz control loop). At 400 kHz, the same read takes about 0.4 ms.

**ACK/NACK debugging**: If the slave does not ACK the address byte, either the address is wrong, the wiring is broken, or the pull-ups are missing. The MPU-6050's 7-bit address is 0x68 (AD0 pin low) or 0x69 (AD0 pin high). A common mistake is using the 8-bit "write address" (0xD0) in the 7-bit address field, which is wrong -- the STM32 HAL shifts the address internally. Use your logic analyzer to verify the exact waveform: you should see the START condition, 7 bits of address, the R/W bit, and then SDA pulled low by the slave for ACK.

### IMU Fundamentals

An Inertial Measurement Unit (IMU) like the MPU-6050 contains a 3-axis accelerometer and a 3-axis gyroscope. The accelerometer measures specific force (gravity plus linear acceleration) and can determine tilt angle when the device is stationary or moving slowly. The gyroscope measures angular velocity (degrees per second) and can be integrated over time to get angular displacement. Each sensor has complementary strengths and weaknesses that make their combination far more useful than either alone.

The accelerometer gives an absolute angle reference (relative to gravity) but is noisy and corrupted by any linear acceleration. If the motor vibrates or the platform accelerates, the accelerometer reading is unreliable in the short term. The gyroscope gives a clean, fast angular rate measurement that integrates into a smooth angle estimate, but it drifts over time because any small bias in the gyro reading accumulates indefinitely through integration. After a few seconds to minutes, the gyro-only angle estimate can be off by tens of degrees.

The **complementary filter** combines both: `angle = alpha * (angle + gyro_rate * dt) + (1 - alpha) * accel_angle`. The alpha parameter (typically 0.95-0.98) determines the time constant of the crossover between gyroscope-dominated short-term response and accelerometer-dominated long-term correction. With alpha = 0.98 and dt = 0.001 (1 kHz sampling), the time constant is approximately dt / (1 - alpha) = 0.05 seconds. This means the accelerometer corrects drift with a time constant of 50 ms, while the gyroscope provides the high-frequency angle information. The complementary filter is mathematically equivalent to a first-order high-pass on the gyro angle plus a first-order low-pass on the accel angle, with matching crossover frequencies.

For computing the accelerometer angle: `accel_angle = atan2(accel_x, accel_z)` gives the roll angle (rotation about the Y axis). Use `atan2` rather than `atan` or `asin` because it handles all four quadrants correctly. The gyroscope reading must be calibrated at startup: average 1000 samples while the device is stationary and subtract that bias from all subsequent readings. Uncalibrated gyro bias is typically 1-5 degrees/second, which causes rapid drift.

### Fixed-Point Arithmetic on MCU

The STM32F411RE has a single-precision floating-point unit (FPU), so for this specific chip, you can use `float` without a performance penalty. However, understanding fixed-point arithmetic matters because many microcontrollers (Cortex-M0, M0+, MSP430, AVR) lack an FPU, and even on the F411RE, double-precision operations are done in software and are extremely slow. Fixed-point is also deterministic (no rounding mode surprises) and uses less memory.

**Q-format notation**: Q16.16 means 16 bits for the integer part and 16 bits for the fractional part, stored in a 32-bit integer. To convert a float to Q16.16: `int32_t q = (int32_t)(float_val * 65536.0f)`. To convert back: `float f = (float)q / 65536.0f`. Multiplication of two Q16.16 numbers requires a 64-bit intermediate: `int32_t result = (int32_t)(((int64_t)a * b) >> 16)`. Addition and subtraction work directly. Division is: `int32_t result = (int32_t)(((int64_t)a << 16) / b)`.

For PID control, the key quantities are: error (angle, typically +/- 180 degrees), gains (Kp, Ki, Kd, which can range over several decades), integral accumulator (can grow large), and output (duty cycle, 0-100%). A Q16.16 format gives a range of +/- 32767 with a resolution of 0.000015, which is sufficient for angle and duty cycle. For the integral accumulator, you may need Q8.24 or even Q0.32 if the gains are small and the accumulator must store fine-grained values. In practice on the F411RE, use `float` and revisit fixed-point only if you port to a cheaper MCU.

### Discrete-Time PID

A continuous PID controller must be discretized for implementation on a microcontroller. The simplest approach is the "position form" using backward Euler for the integral and backward difference for the derivative:

```
e[k] = setpoint - measurement[k]
P[k] = Kp * e[k]
I[k] = I[k-1] + Ki * Ts * e[k]
D[k] = Kd * (measurement[k-1] - measurement[k]) / Ts
output[k] = P[k] + I[k] + D[k]
```

Note that the derivative uses `measurement[k-1] - measurement[k]` (derivative on measurement, negative sign already included) rather than differentiating the error. Ts is the sample period. The integral is a running sum, updated each iteration. This form is called "position form" because the output is the absolute actuator position (duty cycle).

**Sampling rate selection** is determined by the plant dynamics. The Nyquist criterion says you must sample at least twice the bandwidth of the closed-loop system, but in practice 10-20x the bandwidth is recommended for good phase margin in the discrete controller. A small DC motor with no gearbox has a mechanical time constant of 10-50 ms, giving a bandwidth of roughly 20-100 rad/s. A 1 kHz sample rate (Ts = 1 ms) gives 60-300x oversampling, which is plenty. At 1 kHz, the discretization error is negligible and you do not need to worry about Z-transform design -- the continuous gains translate directly.

The "velocity form" is an alternative: `delta_output[k] = Kp * (e[k] - e[k-1]) + Ki * Ts * e[k] + Kd * (e[k] - 2*e[k-1] + e[k-2]) / Ts`. The output is incremental: `output[k] = output[k-1] + delta_output[k]`. The velocity form has inherent anti-windup (just clamp the output after adding the increment) and provides bumpless transfer when changing setpoints or switching from manual to automatic. It is slightly more complex to implement but is preferred in industrial controllers.

### Step Response Analysis

The step response is the system's output when the setpoint changes instantaneously from one value to another (e.g., from 0 degrees to 30 degrees). It reveals all the important dynamic characteristics of the closed-loop system and is the primary tool for evaluating and tuning PID performance.

**Key metrics**: Rise time (Tr) is the time for the output to go from 10% to 90% of the step size. A shorter rise time means faster response. Overshoot is the peak output minus the final value, expressed as a percentage of the step size. For motor control, 5-15% overshoot is typical; for position control, you may want zero overshoot. Settling time (Ts) is the time for the output to stay within a specified band (usually 2% or 5%) of the final value. Steady-state error (Ess) is the final offset between the setpoint and the output; a properly tuned integral term drives this to zero.

**What each PID gain affects**: Increasing Kp reduces rise time and steady-state error but increases overshoot. Increasing Ki eliminates steady-state error but increases overshoot and can cause sustained oscillation. Increasing Kd reduces overshoot and settling time but has no effect on steady-state error and amplifies noise. These are tendencies, not absolute rules -- the actual effect depends on the plant dynamics.

To capture step response data in this project: set up a UART-based command interface that accepts a step command (e.g., "step 30" to command 30 degrees), timestamps each sample using the timer's counter, and prints `timestamp_ms, setpoint, measurement, output` at each control loop iteration. Capture this with a serial terminal logging to a file, then plot it in Python with matplotlib. The plot immediately shows you Tr, overshoot, Ts, and Ess. This data-driven approach is far more productive than staring at the motor and guessing.


## STM32F411RE Specifics

### Timer Peripherals for PWM

The STM32F411RE has the following timer peripherals relevant to PWM generation:

- **TIM1**: Advanced-control timer with complementary outputs and dead-time insertion. 16-bit, up to 100 MHz clock. Best for full H-bridge driving but overkill for TB6612FNG. Uses APB2 clock domain.
- **TIM2**: General-purpose, 32-bit counter. Excellent for long-period timing or encoder counting. APB1 (up to 50 MHz, but with the default clock config the APB1 timer clock is 100 MHz because the prescaler multiplier applies when APB1 prescaler != 1).
- **TIM3, TIM4**: General-purpose, 16-bit. Ideal for PWM generation. Each has 4 channels.
- **TIM5**: General-purpose, 32-bit. Similar to TIM2.
- **TIM9, TIM10, TIM11**: Smaller timers on APB2, 16-bit, 1-2 channels each.

**Recommended configuration for 20 kHz PWM on TIM3**:

With the default clock tree (HSI 16 MHz -> PLL -> SYSCLK 100 MHz, APB1 prescaler = 2, APB1 timer clock = 100 MHz since the hardware doubles the timer clock when APB1 prescaler is not 1):

```
TIM3->PSC  = 0;        // Prescaler: divide by 1 (timer clock = 100 MHz)
TIM3->ARR  = 4999;     // Auto-reload: count 0 to 4999 (5000 steps)
TIM3->CCR1 = 0;        // Start with 0% duty
TIM3->CCMR1 |= (0b110 << 4);  // PWM mode 1 on channel 1 (OC1M bits)
TIM3->CCMR1 |= TIM_CCMR1_OC1PE;  // Preload enable
TIM3->CCER |= TIM_CCER_CC1E;     // Enable channel 1 output
TIM3->CR1  |= TIM_CR1_ARPE;      // Auto-reload preload enable
TIM3->EGR   = TIM_EGR_UG;        // Force update to load preload registers
TIM3->CR1  |= TIM_CR1_CEN;       // Start the timer
```

PWM frequency = 100 MHz / (PSC + 1) / (ARR + 1) = 100 MHz / 1 / 5000 = 20 kHz.

Duty cycle resolution = 1/5000 = 0.02%, which is 12+ effective bits. To set 50% duty: `TIM3->CCR1 = 2500`. To set duty from your PID output (0.0 to 1.0 float): `TIM3->CCR1 = (uint32_t)(pid_output * 4999.0f)`.

**Output compare modes**:
- PWM Mode 1 (OC1M = 110): Output HIGH when counter < CCR, LOW when counter >= CCR (up-counting).
- PWM Mode 2 (OC1M = 111): Inverted version of Mode 1.
- Use center-aligned mode (CMS bits in CR1) if you want symmetric PWM, which reduces current ripple. For this project, edge-aligned (default) is fine.

**Preload registers**: Always enable preload (OC1PE bit) on the CCR and ARPE on the ARR. Without preload, writing a new CCR value takes effect immediately, which can cause a glitch if the counter is currently between the old and new CCR values. With preload, the new value loads at the next update event (counter overflow), guaranteeing a clean transition.

### I2C1/I2C2 Peripheral Registers

The STM32F411RE has three I2C peripherals (I2C1, I2C2, I2C3). For the MPU-6050, use I2C1 (pins PB6/SCL, PB7/SDA) or I2C1 on PB8/PB9 via alternate function remapping.

Key registers for bare-metal I2C configuration:

```
// Enable I2C1 clock
RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

// Configure I2C1 timing for 400 kHz Fast Mode
// Assuming APB1 peripheral clock = 50 MHz (not timer clock)
I2C1->CR2 = 50;                    // APB1 clock in MHz
I2C1->CCR = I2C_CCR_FS            // Fast mode
          | I2C_CCR_DUTY           // Tlow/Thigh = 16/9 (for 400 kHz)
          | 5;                      // CCR value: 50MHz / (25 * 400kHz) = 5
I2C1->TRISE = 16;                  // Max rise time: 300ns * 50MHz + 1 = 16
I2C1->CR1 |= I2C_CR1_PE;          // Enable I2C peripheral
```

**Register-level read sequence for MPU-6050** (address 0x68):

1. Wait until `I2C1->SR2 & I2C_SR2_BUSY` is clear.
2. Set START bit: `I2C1->CR1 |= I2C_CR1_START`.
3. Wait for SB (start bit) flag in SR1. Read SR1 to clear it.
4. Write slave address + write: `I2C1->DR = 0x68 << 1 | 0`.
5. Wait for ADDR flag. Read SR1 then SR2 to clear it.
6. Write register address: `I2C1->DR = register_addr`.
7. Wait for BTF (byte transfer finished).
8. Generate repeated START: `I2C1->CR1 |= I2C_CR1_START`.
9. Wait for SB. Read SR1.
10. Write slave address + read: `I2C1->DR = 0x68 << 1 | 1`.
11. Wait for ADDR. For multi-byte reads, set ACK bit before clearing ADDR.
12. Read bytes from DR, sending ACK after each except the last (send NACK + STOP before reading last byte).

The STM32 I2C peripheral is notoriously tricky at the register level. The HAL abstracts the complex flag-checking sequences. For learning, implement it bare-metal first to understand the protocol, then switch to HAL or LL for the final project if reliability matters.

**DMA integration**: For reading 14 bytes from the MPU-6050 at 1 kHz, DMA is highly recommended. Configure DMA1 Stream 0 (I2C1_RX) to transfer 14 bytes into a buffer, trigger the transfer with I2C, and process the data in the DMA transfer-complete interrupt. This frees the CPU during the ~0.4 ms I2C transaction.

### GPIO Alternate Function for Timers and I2C

Every GPIO pin on the STM32F411RE can be configured for one of 16 alternate functions (AF0-AF15). The mapping is fixed in hardware and documented in the datasheet (Table 9: "Alternate function mapping").

Key mappings for this project:

| Pin  | Alternate Function | Peripheral     |
|------|--------------------|----------------|
| PA6  | AF2                | TIM3_CH1 (PWM) |
| PA7  | AF2                | TIM3_CH2 (PWM) |
| PB6  | AF4                | I2C1_SCL       |
| PB7  | AF4                | I2C1_SDA       |
| PB8  | AF4                | I2C1_SCL (alt) |
| PB9  | AF4                | I2C1_SDA (alt) |
| PA2  | AF7                | USART2_TX      |
| PA3  | AF7                | USART2_RX      |

GPIO configuration for TIM3_CH1 on PA6:

```
// Enable GPIOA clock
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

// Set PA6 to alternate function mode
GPIOA->MODER &= ~(3 << (6 * 2));
GPIOA->MODER |=  (2 << (6 * 2));      // AF mode

// Set alternate function to AF2 (TIM3)
GPIOA->AFR[0] &= ~(0xF << (6 * 4));
GPIOA->AFR[0] |=  (2   << (6 * 4));   // AF2

// Set output speed to high
GPIOA->OSPEEDR |= (2 << (6 * 2));

// No pull-up/pull-down
GPIOA->PUPDR &= ~(3 << (6 * 2));
```

For I2C pins (PB6, PB7), additionally set the output type to open-drain:

```
GPIOB->OTYPER |= (1 << 6) | (1 << 7);  // Open-drain for I2C
```

External pull-up resistors (4.7K to 3.3V) are mandatory for I2C. Do not rely on the internal pull-ups; they are too weak (40-50K) for reliable I2C operation, especially at 400 kHz.

**USART2 note**: On the Nucleo-F411RE, PA2/PA3 are connected to the ST-LINK's virtual COM port. USART2 on these pins gives you USB serial output without any additional hardware. This is your debugging and tuning interface.

### Clock Considerations for Timer Resolution

The default clock configuration on the Nucleo-F411RE (when using the HAL `SystemClock_Config` from STM32CubeMX) is:

- HSI (16 MHz internal RC) or HSE (8 MHz from ST-LINK MCO) as PLL source
- SYSCLK = 100 MHz (via PLL: PLLM=8, PLLN=200, PLLP=4 for HSE; or PLLM=16, PLLN=400, PLLP=4 for HSI)
- AHB = 100 MHz (HPRE = /1)
- APB1 = 50 MHz (PPRE1 = /2) -- but APB1 timer clocks = 100 MHz (hardware 2x when prescaler != 1)
- APB2 = 100 MHz (PPRE2 = /1) -- APB2 timer clocks = 100 MHz

**Timer resolution tradeoffs**:

At 100 MHz timer clock with PSC=0:
- ARR = 4999 -> 20 kHz, 5000 steps (12.3 bits)
- ARR = 9999 -> 10 kHz, 10000 steps (13.3 bits)
- ARR = 2499 -> 40 kHz, 2500 steps (11.3 bits)

Higher PWM frequency means fewer resolution steps. For PID motor control, 12 bits (4096 steps) is more than sufficient. The motor cannot meaningfully respond to changes smaller than about 0.1% duty cycle anyway.

**Timing the control loop**: Use a separate timer (e.g., TIM2 in 32-bit mode) as a microsecond counter for profiling. Configure TIM2 with PSC = 99 (100 MHz / 100 = 1 MHz = 1 us per tick), ARR = 0xFFFFFFFF. Read `TIM2->CNT` before and after your PID computation to measure execution time. Your entire PID loop (IMU read + filter + PID + PWM update) should complete in under 500 us to comfortably fit in a 1 ms (1 kHz) loop.

Use TIM5 or a SysTick-based interrupt at 1 kHz to trigger the control loop. In the interrupt, set a flag; in the main loop, check the flag, run the PID computation, and clear the flag. Never run the PID computation inside the timer interrupt itself -- I2C reads take too long and will block other interrupts.


## How To Build This (Step by Step)

### Step 1: PWM Output Driving Motor at Variable Duty

**What to configure**:
- System clock to 100 MHz (use CubeMX or write clock init manually)
- GPIOA pin 6 as alternate function AF2 (TIM3_CH1)
- TIM3 in PWM mode 1 on channel 1, PSC=0, ARR=4999 for 20 kHz
- A simple main loop that sweeps the duty cycle up and down

**Code approach**:

```c
// In main loop: sweep duty cycle 0% -> 100% -> 0%
for (uint32_t duty = 0; duty <= 4999; duty += 50) {
    TIM3->CCR1 = duty;
    HAL_Delay(20);  // 20 ms between steps
}
for (uint32_t duty = 4999; duty > 0; duty -= 50) {
    TIM3->CCR1 = duty;
    HAL_Delay(20);
}
```

**How to verify**:
- Before connecting the motor, measure the PWM signal on PA6 with your multimeter (should read an average DC voltage proportional to duty cycle) and with the logic analyzer (should show a 20 kHz square wave with the expected duty cycle).
- Verify the frequency is exactly 20 kHz on the logic analyzer. If it is not, your clock configuration is wrong.
- Then connect the motor through the TB6612FNG (next step) and verify it spins faster and slower as duty changes.

**Common pitfalls**:
- Forgetting to enable the TIM3 clock: `RCC->APB1ENR |= RCC_APB1ENR_TIM3EN`.
- Forgetting to set the preload enable bits (OC1PE and ARPE).
- Forgetting `TIM3->EGR = TIM_EGR_UG` to force the first update event, so preloaded values never load.
- Wrong GPIO alternate function number (AF2 for TIM3, not AF1).
- If using CubeMX/HAL, forgetting to call `HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1)`.

### Step 2: TB6612FNG Wiring and Direction Control

**What to configure**:
- Two GPIO pins as push-pull outputs for AIN1 and AIN2 (e.g., PA8 and PA9)
- STBY pin tied to 3.3V (or a third GPIO for emergency stop)
- VM connected to motor supply (4.5-6V from external battery or bench supply; NOT from the Nucleo's 5V pin, which cannot supply enough current)
- VCC connected to Nucleo 3.3V
- GND connected to Nucleo GND AND external supply GND (common ground is mandatory)

**Wiring**:

```
Nucleo PA6 (TIM3_CH1) ---> TB6612FNG PWMA
Nucleo PA8 (GPIO)      ---> TB6612FNG AIN1
Nucleo PA9 (GPIO)      ---> TB6612FNG AIN2
TB6612FNG AO1          ---> Motor terminal 1
TB6612FNG AO2          ---> Motor terminal 2
External 5V supply (+) ---> TB6612FNG VM
External 5V supply (-) ---> TB6612FNG GND ---> Nucleo GND
Nucleo 3.3V            ---> TB6612FNG VCC
3.3V through 10K       ---> TB6612FNG STBY
```

Place a 100nF ceramic capacitor between VM and GND, as close to the TB6612FNG as possible. Place another 100nF between VCC and GND.

**Code approach**:

```c
void motor_set(float effort) {
    // effort: -1.0 (full reverse) to +1.0 (full forward)
    if (effort > 0.0f) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   // AIN1 = HIGH
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); // AIN2 = LOW
        TIM3->CCR1 = (uint32_t)(effort * 4999.0f);
    } else if (effort < 0.0f) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); // AIN1 = LOW
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);   // AIN2 = HIGH
        TIM3->CCR1 = (uint32_t)(-effort * 4999.0f);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   // AIN1 = HIGH
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);   // AIN2 = HIGH (brake)
        TIM3->CCR1 = 4999;  // Full duty for brake mode
    }
}
```

**How to verify**:
- Call `motor_set(0.3)` and verify motor spins in one direction.
- Call `motor_set(-0.3)` and verify it reverses.
- Call `motor_set(0.0)` and verify the motor brakes (stops quickly, not coasts).
- Measure current draw with the multimeter in series. A 130 motor at 5V no-load should draw 70-150 mA.

**Common pitfalls**:
- No common ground between external supply and Nucleo. The motor will not respond or will behave erratically.
- STBY pin floating. It must be HIGH for operation.
- Trying to power the motor from the Nucleo's USB 5V rail. The USB port provides at most 500 mA, and motor startup current spikes can cause the Nucleo to brown out and reset.
- Motor supply voltage too high for the 130-type motor (rated 3-6V). Do not feed it 12V.

### Step 3: I2C Driver for MPU-6050

**What to configure**:
- GPIOB pins 6 (SCL) and 7 (SDA) as alternate function AF4, open-drain, with external 4.7K pull-ups to 3.3V
- I2C1 peripheral in fast mode (400 kHz)
- MPU-6050: VCC to 3.3V, GND to GND, AD0 to GND (address = 0x68), SDA and SCL to PB7 and PB6 respectively

**Code approach**:

```c
// Read WHO_AM_I register (0x75) to verify communication
uint8_t who_am_i;
HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x75, I2C_MEMADD_SIZE_8BIT,
                 &who_am_i, 1, 100);
// who_am_i should be 0x68

// Wake up MPU-6050 (it starts in sleep mode)
uint8_t pwr_mgmt = 0x00;  // Clear sleep bit, use internal 8 MHz oscillator
HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x6B, I2C_MEMADD_SIZE_8BIT,
                  &pwr_mgmt, 1, 100);

// Configure gyroscope: +/- 250 deg/s (most sensitive)
uint8_t gyro_config = 0x00;  // FS_SEL = 0
HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x1B, I2C_MEMADD_SIZE_8BIT,
                  &gyro_config, 1, 100);

// Configure accelerometer: +/- 2g (most sensitive)
uint8_t accel_config = 0x00;  // AFS_SEL = 0
HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x1C, I2C_MEMADD_SIZE_8BIT,
                  &accel_config, 1, 100);

// Configure DLPF (Digital Low Pass Filter): ~44 Hz bandwidth
uint8_t dlpf_config = 0x03;
HAL_I2C_Mem_Write(&hi2c1, 0x68 << 1, 0x1A, I2C_MEMADD_SIZE_8BIT,
                  &dlpf_config, 1, 100);

// Read accel + temp + gyro (14 bytes starting at 0x3B)
uint8_t raw_data[14];
HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, 0x3B, I2C_MEMADD_SIZE_8BIT,
                 raw_data, 14, 100);

// Parse (big-endian, 16-bit signed):
int16_t accel_x = (raw_data[0] << 8) | raw_data[1];
int16_t accel_y = (raw_data[2] << 8) | raw_data[3];
int16_t accel_z = (raw_data[4] << 8) | raw_data[5];
// raw_data[6..7] is temperature, skip
int16_t gyro_x  = (raw_data[8] << 8) | raw_data[9];
int16_t gyro_y  = (raw_data[10] << 8) | raw_data[11];
int16_t gyro_z  = (raw_data[12] << 8) | raw_data[13];

// Convert to physical units:
float ax = accel_x / 16384.0f;  // g (at +/- 2g range)
float gy = gyro_x / 131.0f;     // deg/s (at +/- 250 deg/s range)
```

**How to verify**:
- WHO_AM_I register read returns 0x68. If it returns 0x00 or 0xFF, the I2C bus is not working.
- Use the logic analyzer on SCL/SDA to see the actual I2C waveform. Verify START conditions, address byte, ACK bits, and data bytes.
- Print accelerometer data over UART while tilting the sensor. At rest, one axis should read approximately +/- 1g (16384 raw at +/- 2g range) and the others near zero, depending on orientation.
- Print gyro data while rotating the sensor. At rest, readings should be near zero (within +/- 5 LSB after bias calibration).

**Common pitfalls**:
- Missing external pull-up resistors. The I2C bus will not work with only internal pull-ups.
- Using the 8-bit address (0xD0) instead of the 7-bit address (0x68). The HAL shifts left by 1 internally when you pass the address; check your HAL version's documentation.
- Not waking up the MPU-6050. Register 0x6B defaults to 0x40 (sleep mode). You must write 0x00 to it.
- I2C bus lockup: if the MCU resets during an I2C transaction, the slave may hold SDA low indefinitely. To recover, toggle SCL manually (set PB6 as GPIO output, toggle 9 times, then reconfigure as AF). Add this recovery sequence to your init code.

### Step 4: Complementary Filter for Angle Estimation

**What to configure**:
- A timer interrupt or SysTick at 1 kHz for consistent sampling
- Gyroscope bias calibration at startup (1000 samples average while stationary)

**Code approach**:

```c
// At startup: calibrate gyro bias
float gyro_bias_x = 0.0f;
for (int i = 0; i < 1000; i++) {
    read_mpu6050(raw_data);
    int16_t gyro_x_raw = (int16_t)((uint16_t)raw_data[8] << 8 | raw_data[9]);
    gyro_bias_x += gyro_x_raw / 131.0f;
    HAL_Delay(1);
}
gyro_bias_x /= 1000.0f;

// In 1 kHz loop:
float dt = 0.001f;
float alpha = 0.98f;
float angle = 0.0f;  // Initialize to accel angle on first iteration

void update_angle(void) {
    read_mpu6050(raw_data);

    float accel_x = (int16_t)((uint16_t)raw_data[0] << 8 | raw_data[1]) / 16384.0f;
    float accel_z = (int16_t)((uint16_t)raw_data[4] << 8 | raw_data[5]) / 16384.0f;
    float accel_angle = atan2f(accel_x, accel_z) * 57.2958f;  // rad to deg

    float gyro_rate = (int16_t)((uint16_t)raw_data[8] << 8 | raw_data[9]) / 131.0f;
    gyro_rate -= gyro_bias_x;

    angle = alpha * (angle + gyro_rate * dt) + (1.0f - alpha) * accel_angle;
}
```

**How to verify**:
- Print the angle over UART at 10-50 Hz (not 1 kHz, that will flood the serial port).
- Tilt the sensor slowly from 0 to 90 degrees and back. The angle should track smoothly.
- Tilt it quickly. The gyroscope should keep the angle accurate during fast motion while the accelerometer prevents long-term drift.
- Leave the sensor stationary for 5 minutes. The angle should not drift more than 0.5 degrees. If it drifts, your gyro bias calibration is bad or alpha is too high.

**Common pitfalls**:
- Using `atan2(accel_y, accel_z)` when you meant roll, or `atan2(accel_x, accel_z)` when you meant pitch. Determine your axis convention first and be consistent.
- Forgetting to convert `atan2f` output from radians to degrees (multiply by 180/pi = 57.2958).
- Not calibrating gyro bias. Even 1 deg/s of uncorrected bias causes 60 degrees of drift per minute.
- Alpha too high (0.999): the filter trusts the gyro almost exclusively and will drift. Alpha too low (0.5): the filter is noisy from accelerometer vibration. Start with 0.98 and adjust.
- Computing the complementary filter at inconsistent intervals. If `dt` varies, the gyro integration accumulates error. Use a hardware timer interrupt to guarantee consistent timing.

### Step 5: PID Loop with P-Only Control

**What to configure**:
- A target angle setpoint (start with 0 degrees = balanced upright)
- PID struct with only Kp active
- Connect the PID output to the motor via `motor_set()`

**Code approach**:

```c
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_measurement;
    float output_min, output_max;
} PID_t;

float pid_update(PID_t *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;

    // P-only for now
    float output = pid->Kp * error;

    // Clamp output
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    pid->prev_measurement = measurement;
    return output;
}

// In main:
PID_t pid = {
    .Kp = 0.01f,  // Start very small
    .Ki = 0.0f,
    .Kd = 0.0f,
    .integral = 0.0f,
    .prev_measurement = 0.0f,
    .output_min = -1.0f,
    .output_max = 1.0f
};

// In 1 kHz loop:
float angle = get_filtered_angle();
float effort = pid_update(&pid, target_angle, angle, 0.001f);
motor_set(effort);
```

**How to verify**:
- With a very small Kp, tilt the platform. The motor should push gently in the correcting direction.
- Slowly increase Kp. The correction should become more aggressive.
- At some Kp, the system will oscillate. Record this Kp value as an approximation of Ku (critical gain) for Ziegler-Nichols if desired.
- Back off Kp to about 60% of the oscillation value.
- Note the steady-state error: with P-only control, the motor will not hold the exact setpoint. There will be a constant offset (droop). This is expected and motivates the integral term.

**Common pitfalls**:
- Starting with Kp too large. The motor will slam full-speed in one direction and may damage the mechanical setup. Start with Kp = 0.001 and increase by 2x each test.
- Wrong sign on the error or motor direction. If the motor pushes in the wrong direction (positive feedback instead of negative feedback), the system diverges instantly. Flip the motor wires or negate the PID output.
- Not clamping the output. Without clamping, a large error produces a large output that overflows the duty cycle calculation or drives the motor function with values outside [-1, 1].

### Step 6: Add Integral Term with Anti-Windup

**What to configure**:
- Add integral accumulation to the PID struct
- Implement integral clamping anti-windup

**Code approach**:

```c
float pid_update(PID_t *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;

    // Proportional
    float P = pid->Kp * error;

    // Integral with anti-windup (clamping)
    float output_unsat = P + pid->Ki * (pid->integral + error * dt);

    // Only accumulate integral if output is not saturated,
    // or if the error is driving the integral toward zero
    if (output_unsat >= pid->output_min && output_unsat <= pid->output_max) {
        pid->integral += error * dt;
    } else if ((output_unsat > pid->output_max && error < 0.0f) ||
               (output_unsat < pid->output_min && error > 0.0f)) {
        // Error is unwinding the integral, allow it
        pid->integral += error * dt;
    }

    float I = pid->Ki * pid->integral;

    float output = P + I;

    // Clamp final output
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    pid->prev_measurement = measurement;
    return output;
}
```

**How to verify**:
- Start with Ki very small (e.g., Ki = 0.001).
- Apply a constant disturbance (e.g., gently push the platform off-balance with your finger). With P-only, the system holds with an offset. With PI, the system should slowly correct back to zero offset.
- Remove the disturbance. The system should return to the setpoint without excessive overshoot.
- Test anti-windup: hold the motor shaft still (stall condition) for 5 seconds, then release. Without anti-windup, the motor will slam full-speed for several seconds as the integral unwinds. With anti-windup, recovery should be fast (under 1 second).

**Common pitfalls**:
- Ki too large. The integral term responds slowly by design. If Ki is large, it overshoots massively and oscillates at a low frequency (much slower than P-term oscillations).
- Not testing windup. Your anti-windup code may have bugs that only manifest during saturation events. Deliberately stall the motor to test.
- Integral accumulator overflow. If using fixed-point, the integral can grow unbounded. Even with float, add a hard clamp on the integral value itself (e.g., +/- 1000) as a safety measure.

### Step 7: Add Derivative Term with Filtering

**What to configure**:
- Derivative computed on measurement (not error)
- First-order low-pass filter on the derivative
- Filter coefficient N = 10 as a starting point

**Code approach**:

```c
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_measurement;
    float prev_derivative;  // Filtered derivative state
    float output_min, output_max;
    float N;  // Derivative filter coefficient
} PID_t;

float pid_update(PID_t *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;

    // Proportional
    float P = pid->Kp * error;

    // Integral (with anti-windup as in Step 6)
    pid->integral += error * dt;
    float I = pid->Ki * pid->integral;

    // Derivative on measurement with filtering
    float raw_derivative = -(measurement - pid->prev_measurement) / dt;
    float alpha_d = pid->Kd / (pid->Kd + pid->N * dt * pid->Kp);
    // Simpler alternative: fixed filter alpha
    float filter_alpha = 0.8f;  // Adjustable, 0.0 = no filtering, 0.99 = heavy filtering
    float filtered_derivative = filter_alpha * pid->prev_derivative
                              + (1.0f - filter_alpha) * raw_derivative;
    float D = pid->Kd * filtered_derivative;

    pid->prev_derivative = filtered_derivative;
    pid->prev_measurement = measurement;

    float output = P + I + D;

    // Anti-windup: clamp integral if output saturates
    if (output > pid->output_max) {
        pid->integral -= error * dt;  // Undo integration
        output = pid->output_max;
    } else if (output < pid->output_min) {
        pid->integral -= error * dt;
        output = pid->output_min;
    }

    return output;
}
```

**How to verify**:
- With Kd = 0, observe the system's overshoot to a step input.
- Add a small Kd (start with Kd = Kp * 0.01). Overshoot should decrease.
- Increase Kd. At some point, the system becomes jittery due to noise amplification (even with filtering). This is the practical upper limit.
- Observe the filtered derivative value on UART. It should be smooth. Compare it with the unfiltered derivative to see how much noise the filter removes.

**Common pitfalls**:
- Computing derivative on error instead of measurement. When the setpoint changes, you get a massive derivative spike (derivative kick).
- No filter on the derivative. The derivative of a quantized, noisy signal is extremely noisy. The motor will vibrate and buzz.
- Filter alpha too high (0.99) introduces too much phase lag in the derivative, negating its damping effect. Start with 0.8.
- Division by dt in the derivative: if dt is ever zero (e.g., timer interrupt timing issue), you get division by zero. Add a guard: `if (dt < 0.0001f) dt = 0.001f;`.

### Step 8: UART CLI for Live Kp/Ki/Kd Tuning

**What to configure**:
- USART2 at 115200 baud (connected to ST-LINK virtual COM port via PA2/PA3)
- A simple command parser that accepts text commands
- Ring buffer for non-blocking UART reception

**Code approach**:

Implement a minimal command-line interface that accepts commands like:
```
kp 0.05       -> Set Kp to 0.05
ki 0.002      -> Set Ki to 0.002
kd 0.001      -> Set Kd to 0.001
sp 30.0       -> Set setpoint to 30.0 degrees
step 20.0     -> Apply a step of 20.0 degrees and log response
log on        -> Start logging setpoint, angle, output at 100 Hz
log off       -> Stop logging
reset         -> Reset integral accumulator, PID state
```

```c
// UART receive interrupt fills a ring buffer
char rx_buf[256];
volatile uint16_t rx_head = 0, rx_tail = 0;
volatile float setpoint = 0.0f;  // Modified from CLI, read from control loop

void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) {
        rx_buf[rx_head++ & 0xFF] = USART2->DR;
    }
}

// In main loop (not in interrupt): parse complete lines
void process_commands(void) {
    static char cmd_buf[64];
    static uint8_t cmd_len = 0;

    while ((rx_tail & 0xFF) != (rx_head & 0xFF)) {
        char c = rx_buf[rx_tail++ & 0xFF];
        if (c == '\n' || c == '\r') {
            cmd_buf[cmd_len] = '\0';
            parse_command(cmd_buf);
            cmd_len = 0;
        } else if (cmd_len < 63) {
            cmd_buf[cmd_len++] = c;
        }
    }
}

void parse_command(const char *cmd) {
    float val;
    if (sscanf(cmd, "kp %f", &val) == 1) {
        pid.Kp = val;
        printf("Kp = %.4f\r\n", pid.Kp);
    } else if (sscanf(cmd, "ki %f", &val) == 1) {
        pid.Ki = val;
        pid.integral = 0.0f;  // Reset integral when changing Ki
        printf("Ki = %.4f\r\n", pid.Ki);
    } else if (sscanf(cmd, "kd %f", &val) == 1) {
        pid.Kd = val;
        printf("Kd = %.4f\r\n", pid.Kd);
    } else if (sscanf(cmd, "sp %f", &val) == 1) {
        setpoint = val;
        printf("Setpoint = %.2f\r\n", setpoint);
    } else if (strncmp(cmd, "log on", 6) == 0) {
        logging_enabled = 1;
    } else if (strncmp(cmd, "log off", 7) == 0) {
        logging_enabled = 0;
    }
}
```

**How to verify**:
- Connect a serial terminal (PuTTY, minicom, or picocom) at 115200 baud to the Nucleo's COM port.
- Type `kp 0.01` and press Enter. The system should confirm the new value and the motor behavior should change immediately.
- Type `log on` and see streaming data. Verify the data rate is correct (e.g., one line per 10 ms for 100 Hz logging).

**Common pitfalls**:
- Blocking `printf` inside the control loop ISR. Never use printf (or any slow function) in a time-critical context. Buffer the output and print from the main loop.
- UART buffer overflow when logging at high rates. At 115200 baud, you get about 11520 characters per second. A log line of 40 characters at 100 Hz = 4000 char/s, which fits. At 1000 Hz (40000 char/s), it does not. Log at 100 Hz or less.
- Not resetting the integral accumulator when changing Ki. The integral was accumulated with the old Ki scaling; the new Ki applies to the old sum, producing a sudden output jump.
- Forgetting to retarget `printf` to USART2. In STM32, you need to implement `_write()` or `__io_putchar()` to redirect stdout to UART.

### Step 9: Step Response Capture and Logging

**What to configure**:
- A "step" command that changes the setpoint by a specified amount
- High-resolution timestamped logging that captures every control loop iteration for the duration of the transient
- A fixed-duration capture (e.g., 2 seconds at 1 kHz = 2000 samples)

**Code approach**:

```c
#define LOG_BUFFER_SIZE 2000

typedef struct {
    uint32_t timestamp_us;
    float setpoint;
    float measurement;
    float output;
} log_entry_t;

log_entry_t log_buffer[LOG_BUFFER_SIZE];
volatile uint16_t log_index = 0;
volatile uint8_t log_capturing = 0;

// Called in 1 kHz control loop
void capture_sample(float sp, float meas, float out) {
    if (log_capturing && log_index < LOG_BUFFER_SIZE) {
        log_buffer[log_index].timestamp_us = TIM2->CNT;  // Microsecond timer
        log_buffer[log_index].setpoint = sp;
        log_buffer[log_index].measurement = meas;
        log_buffer[log_index].output = out;
        log_index++;
        if (log_index >= LOG_BUFFER_SIZE) {
            log_capturing = 0;  // Stop capturing
        }
    }
}

// Called from CLI when "step" command received
void start_step_response(float step_size) {
    log_index = 0;
    log_capturing = 1;
    setpoint += step_size;  // Apply step
}

// Called from main loop to dump captured data over UART
void dump_log(void) {
    if (!log_capturing && log_index > 0) {
        printf("time_us,setpoint,measurement,output\r\n");
        for (uint16_t i = 0; i < log_index; i++) {
            printf("%lu,%.3f,%.3f,%.3f\r\n",
                   log_buffer[i].timestamp_us,
                   log_buffer[i].setpoint,
                   log_buffer[i].measurement,
                   log_buffer[i].output);
        }
        log_index = 0;  // Clear after dump
    }
}
```

**Python analysis script**:

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("step_response.csv")
df["time_ms"] = (df["time_us"] - df["time_us"].iloc[0]) / 1000.0

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
ax1.plot(df["time_ms"], df["setpoint"], "r--", label="Setpoint")
ax1.plot(df["time_ms"], df["measurement"], "b-", label="Measurement")
ax1.set_ylabel("Angle (deg)")
ax1.legend()

ax2.plot(df["time_ms"], df["output"], "g-", label="Control Output")
ax2.set_ylabel("Output")
ax2.set_xlabel("Time (ms)")
ax2.legend()

plt.tight_layout()
plt.savefig("step_response.png")
plt.show()
```

**How to verify**:
- Issue `step 20` command. Wait 2 seconds. Issue `dump` command.
- Copy the CSV data from the serial terminal to a file.
- Run the Python script. The plot should show a clear step response with the setpoint jumping and the measurement following.
- Measure overshoot, rise time, and settling time directly from the plot.
- Compare different Kp/Ki/Kd combinations. Save each plot for comparison.

**Common pitfalls**:
- Logging too many samples (RAM overflow). 2000 samples of 16 bytes each = 32 KB. The STM32F411RE has 128 KB of RAM, so this is fine. But do not try to log 10,000 samples.
- Timestamps wrapping around. A 32-bit microsecond timer wraps every ~71 minutes. If your capture spans less than that, no issue. Use relative timestamps (subtract the first sample's time) in your analysis.
- Serial terminal not capturing data correctly. Use a terminal that can log to a file (minicom -C logfile.txt, or PuTTY session logging).

### Step 10: OLED Real-Time Display

**What to configure**:
- SSD1306 128x64 OLED via I2C (typically address 0x3C)
- Wired to the same I2C bus as the MPU-6050 (both devices on I2C1, different addresses)
- A display update rate of 10-20 Hz (updating a 128x64 OLED over I2C at 400 kHz takes about 2 ms for a full frame, and you do not need more than 20 fps for a status display)

**Warning**: The SSD1306 and MPU-6050 share the I2C bus. The MPU-6050 reads (14 bytes at 1 kHz) are time-critical for the control loop. The SSD1306 writes (1024 bytes for a full frame) are not. You must ensure that OLED updates do not block or delay the MPU-6050 reads. The simplest approach: update the OLED from the main loop in small chunks (e.g., 32 bytes per main loop iteration), yielding the bus for the control loop's MPU-6050 reads in between. Alternatively, use a second I2C bus (I2C2 on PB10/PB3) for the OLED.

**Code approach** (using a typical SSD1306 library):

```c
// Display layout:
// Line 0: "PID Motor Control"
// Line 1: "Angle: XX.X deg"
// Line 2: "SP: XX.X  Out: X.XX"
// Line 3: "Kp:X.XX Ki:X.XX Kd:X.XX"

void update_display(void) {
    static uint32_t last_update = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_update < 100) return;  // 10 Hz update
    last_update = now;

    char line[22];  // 128 pixels / 6 pixels per char = 21 chars max

    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("PID Motor Ctrl", Font_6x8, White);

    snprintf(line, sizeof(line), "Angle: %6.1f deg", angle);
    ssd1306_SetCursor(0, 16);
    ssd1306_WriteString(line, Font_6x8, White);

    snprintf(line, sizeof(line), "SP:%5.1f Out:%5.2f", setpoint, pid_output);
    ssd1306_SetCursor(0, 32);
    ssd1306_WriteString(line, Font_6x8, White);

    snprintf(line, sizeof(line), "P%.2f I%.3f D%.3f", pid.Kp, pid.Ki, pid.Kd);
    ssd1306_SetCursor(0, 48);
    ssd1306_WriteString(line, Font_6x8, White);

    ssd1306_UpdateScreen();  // Flush framebuffer to display
}
```

**How to verify**:
- The display should show live angle and PID values, updating smoothly at ~10 Hz.
- Tilt the sensor and verify the displayed angle matches the UART output.
- Change PID gains via UART CLI and verify the display updates.
- Most importantly: verify that the control loop timing is not degraded. Measure the loop period with TIM2 -- it should still be consistently 1 ms +/- 0.01 ms. If the OLED update causes jitter, move it to a lower-priority context or use a separate I2C bus.

**Common pitfalls**:
- I2C address conflict. The SSD1306 is typically at 0x3C (or 0x3D). The MPU-6050 is at 0x68. These do not conflict, but verify with a bus scan if things do not work.
- Blocking `ssd1306_UpdateScreen()` call taking 2+ ms and causing the control loop to miss a deadline. Profile it. If it is too slow, update partial rows per iteration or use DMA.
- `snprintf` with floating-point on STM32. By default, the nano spec C library does not support `%f` in printf/snprintf. You must enable "Use float with printf" in the linker settings (add `-u _printf_float` to linker flags).


## Materials Needed

### Hardware List

| Item | Specification | Quantity | Notes |
|------|--------------|----------|-------|
| Nucleo-F411RE | STM32F411RE development board | 1 | Has built-in ST-LINK debugger and virtual COM port |
| TB6612FNG breakout | Dual H-bridge motor driver, SparkFun or Adafruit | 1 | Handles up to 1.2A per channel |
| DC motor (130 type) | 3-6V, with gearbox if available | 1 | A gearbox makes the response slower and easier to control |
| MPU-6050 breakout | GY-521 module (includes pull-ups and regulator) | 1 | 3.3V or 5V tolerant |
| SSD1306 OLED | 128x64, I2C, 0.96 inch | 1 | 4-pin I2C version (VCC, GND, SCL, SDA) |
| SG90 servo | Standard micro servo | 1-2 | For future extensions (pan/tilt) |
| Breadboard | Full-size (830 tie points) | 2 | One for driver circuit, one for sensors |
| Jumper wires | Male-male and male-female | ~40 | Various lengths |
| Resistor kit | 1/4W through-hole | 1 | Need 4.7K for I2C pull-ups, 10K for STBY pull-up |
| Capacitor, ceramic | 100nF (0.1uF) | 4 | Decoupling for TB6612FNG VM, VCC, and sensor VCC |
| Capacitor, electrolytic | 100uF, 16V or higher | 1 | Bulk decoupling on motor power supply |
| External power supply | 5V, 1A+ (bench supply, USB charger, or 4xAA batteries) | 1 | For motor power; do NOT share with Nucleo USB |
| USB Micro-B cable | For Nucleo programming and serial | 1 | Usually comes with the Nucleo |
| Multimeter | Basic digital | 1 | For voltage/current measurements |
| Logic analyzer | 8-channel, compatible with PulseView/sigrok | 1 | For I2C and PWM debugging |

### Wiring Diagram Description

**Power connections**:
```
External 5V (+) ----+---- [100uF electrolytic] ---- External 5V (-)
                    |
                    +---- TB6612FNG VM
                    |
                    +---- [100nF ceramic] ---- GND

Nucleo 3.3V --------+---- TB6612FNG VCC
                    |
                    +---- [100nF ceramic] ---- GND
                    |
                    +---- MPU-6050 VCC (if 3.3V module)
                    |
                    +---- SSD1306 VCC
                    |
                    +---- [4.7K resistor] ---- I2C SCL line (PB6)
                    |
                    +---- [4.7K resistor] ---- I2C SDA line (PB7)

All GND connections tied together:
  Nucleo GND = External supply GND = TB6612FNG GND = MPU-6050 GND = SSD1306 GND
```

**Signal connections**:
```
Nucleo PA6 (TIM3_CH1)  ----> TB6612FNG PWMA
Nucleo PA8 (GPIO out)   ----> TB6612FNG AIN1
Nucleo PA9 (GPIO out)   ----> TB6612FNG AIN2
TB6612FNG AO1           ----> Motor terminal 1
TB6612FNG AO2           ----> Motor terminal 2
TB6612FNG STBY          ----> 3.3V (through 10K resistor, or direct)

Nucleo PB6 (I2C1_SCL)  ----> MPU-6050 SCL ----> SSD1306 SCL
Nucleo PB7 (I2C1_SDA)  ----> MPU-6050 SDA ----> SSD1306 SDA
MPU-6050 AD0           ----> GND (address = 0x68)

Nucleo PA2 (USART2_TX) ----> (internal to ST-LINK, no external wiring needed)
Nucleo PA3 (USART2_RX) ----> (internal to ST-LINK, no external wiring needed)
```

**Important notes on wiring**:
- The GY-521 MPU-6050 breakout module typically includes 2.2K pull-up resistors on SDA and SCL. If you are also adding external 4.7K pull-ups, the effective pull-up resistance is about 1.5K, which is fine. If the SSD1306 module also has pull-ups, you may end up with too-strong pull-ups (under 1K), which wastes power but usually still works. If I2C is unreliable, remove the external pull-ups and rely on the module's built-in ones.
- Keep motor wires away from I2C/sensor wires. Motor switching generates EMI that can corrupt I2C communication. A 100nF capacitor across the motor terminals helps suppress brush noise.
- The TB6612FNG has separate GND pins for motor ground and logic ground. Connect both to the common ground.


## Interview Questions

### Junior Level

**Q1: What does each term in a PID controller do? When would you use only PI instead of full PID?**

The proportional term (P) produces an output proportional to the current error -- it reacts to how far off the system is right now. The integral term (I) accumulates past error over time, which eliminates steady-state offset but responds slowly and can overshoot. The derivative term (D) responds to the rate of change of the error, providing a damping/predictive effect that reduces overshoot.

You would use PI (no D) when the measurement is noisy and the derivative term would amplify that noise unacceptably, or when the plant is naturally well-damped and derivative action is unnecessary. Many industrial process control loops (temperature, flow, level) use PI only because the processes are slow and noisy. You would add D when the system has significant inertia and tends to overshoot, such as position control of a motor or balancing systems.

**Q2: What is integral windup and how do you prevent it?**

Integral windup occurs when the controller output saturates (e.g., the motor is already at maximum speed) but the integral term keeps accumulating error. When the error eventually changes sign, the bloated integral must unwind before the controller can respond, causing massive overshoot and sluggish recovery.

Prevention methods include: (1) Integral clamping -- stop accumulating the integral when the output is saturated. (2) Back-calculation -- feed the difference between the saturated and unsaturated output back to reduce the integral. (3) Conditional integration -- only integrate when the error is small enough that the output is not saturated. The simplest implementation is clamping: after computing the output, if it exceeds the actuator limits, clamp it and do not update the integral accumulator.

**Q3: Explain I2C communication. What happens on the bus when a master reads one byte from a slave?**

I2C uses two wires: SCL (clock, driven by master) and SDA (data, driven by either master or slave depending on the phase). Both are open-drain with pull-up resistors. To read one byte from a slave at register address R:

1. Master sends START condition (SDA falls while SCL is high).
2. Master sends 7-bit slave address with write bit (bit 0 = 0). Slave pulls SDA low for ACK.
3. Master sends the register address byte (R). Slave ACKs.
4. Master sends repeated START.
5. Master sends 7-bit slave address with read bit (bit 0 = 1). Slave ACKs.
6. Slave drives SDA with the data byte (8 bits, MSB first), clocked by master's SCL.
7. Master sends NACK (does not pull SDA low) to indicate it is done reading.
8. Master sends STOP condition (SDA rises while SCL is high).

The repeated START in step 4 is necessary to switch the bus direction without releasing it (which would allow another master to arbitrate).

**Q4: Why do we use PWM to control motor speed instead of simply varying the voltage?**

PWM is efficient because the transistors in the driver are either fully on (low resistance, low power dissipation) or fully off (no current, no dissipation). A linear voltage regulator would dissipate power as heat in the pass transistor, with efficiency proportional to Vout/Vin. At 50% speed, PWM wastes almost no power in the driver, while a linear approach wastes roughly 50% as heat.

Additionally, PWM provides better low-speed torque than a reduced DC voltage because during the "on" phase, the motor sees the full supply voltage and draws more current through its winding inductance. The motor's inductance smooths the current, so the motor effectively sees a lower average voltage but with current pulses that maintain torque at low speeds. A linearly reduced voltage results in proportionally reduced current and therefore reduced torque.

**Q5: What is the difference between an accelerometer and a gyroscope? Why use both?**

An accelerometer measures specific force (linear acceleration including gravity). When stationary, it reads gravity and can determine tilt angle. A gyroscope measures angular velocity (rotation rate in degrees per second).

You use both because each compensates for the other's weakness. The accelerometer provides an absolute angle reference (relative to gravity) but is noisy and corrupted by vibration or linear acceleration. The gyroscope provides smooth, fast angular rate data that integrates to a clean angle estimate, but it drifts over time due to small bias errors that accumulate through integration. A complementary filter or Kalman filter combines the two: the gyroscope dominates short-term angle estimation (fast, smooth) while the accelerometer corrects long-term drift.

**Q6: What is the purpose of a decoupling capacitor, and where should it be placed?**

A decoupling capacitor provides a local charge reservoir for an IC, supplying instantaneous current during fast logic transitions without requiring it to travel through long PCB traces from the power supply. Without decoupling, fast current transients cause voltage dips on the power rail, which can corrupt logic, cause resets, or generate EMI.

Place decoupling capacitors as physically close to the IC's power pins as possible, with the shortest trace length achievable. The typical values are 100nF ceramic for high-frequency decoupling (resonant at tens of MHz) and optionally 1-10uF ceramic for lower-frequency decoupling. For motor drivers, add a larger electrolytic (100uF+) near the motor power input to handle the large current transients when the motor starts and stops.

**Q7: What is the auto-reload register (ARR) in an STM32 timer, and how does it determine PWM frequency?**

The ARR sets the maximum value the timer counter counts to before resetting to zero (in up-counting mode). The timer counter increments on each timer clock tick. When the counter reaches ARR, it overflows back to zero, and this period defines the PWM frequency.

PWM frequency = Timer_Clock / (PSC + 1) / (ARR + 1), where PSC is the prescaler value. For example, with a 100 MHz timer clock, PSC = 0, and ARR = 4999, the frequency is 100 MHz / 1 / 5000 = 20 kHz. The duty cycle is set by the CCR (capture/compare register): duty = CCR / (ARR + 1). Higher ARR values give lower frequency but finer duty cycle resolution (more steps between 0% and 100%).

**Q8: You connect an I2C device and it does not respond (no ACK). What do you check?**

Systematic debugging approach:

1. **Wiring**: Verify SDA and SCL are connected to the correct pins, not swapped. Verify the device is powered (measure VCC at the device).
2. **Pull-ups**: Measure SCL and SDA with a multimeter -- both should read close to VCC when idle. If they are at 0V, pull-up resistors are missing or the bus is held low.
3. **Address**: Verify the 7-bit address is correct. Common mistake: using the 8-bit write address (left-shifted) in a function that expects 7-bit. Check the AD0 pin state if the device has configurable address bits.
4. **Logic analyzer**: Capture the actual bus waveform. Verify START condition, correct address bits, R/W bit, and look for ACK (SDA pulled low on the 9th clock cycle). If ACK is missing but the waveform looks correct, the device may be in sleep mode or damaged.
5. **Bus scan**: Run an I2C scan (send each possible address and check for ACK) to see if any device responds. If a device responds at a different address, you had the address wrong.
6. **Bus lockup**: If SDA is stuck low, the slave may be holding it from an interrupted transaction. Toggle SCL manually (9 pulses as GPIO) to reset the slave's state machine.

### Mid Level

**Q9: Explain the Ziegler-Nichols tuning method. What are its limitations?**

The Ziegler-Nichols ultimate gain method works as follows: disable the integral and derivative terms (Ki = Kd = 0). Gradually increase Kp until the system reaches sustained, constant-amplitude oscillation. This critical gain is Ku, and the oscillation period is Tu. The Z-N table provides starting gains: Kp = 0.6*Ku, Ti = Tu/2 (so Ki = Kp/Ti = 1.2*Ku/Tu), Td = Tu/8 (so Kd = Kp*Td = 0.075*Ku*Tu).

Limitations: (1) It requires bringing the system to marginal stability, which is dangerous for mechanical systems that could be damaged by oscillation. (2) The resulting gains are aggressive, producing about 25% overshoot, which may be unacceptable. (3) It assumes the plant can be adequately modeled as a system with a single dominant time constant, which is not always true. (4) It does not account for actuator saturation, time delays, or nonlinearities. (5) Many real systems (overdamped plants, integrating plants) may not exhibit sustained oscillation for any finite gain.

In practice, Z-N provides a starting point that requires further manual refinement. Alternative methods like Cohen-Coon (based on step response) or relay auto-tuning (which limits the oscillation amplitude) are often preferred.

**Q10: What is derivative kick and how is it handled in practice?**

Derivative kick occurs when the setpoint changes abruptly (a step change). The error e(t) = setpoint - measurement changes instantaneously, producing a theoretically infinite derivative de/dt. In discrete time, the derivative term becomes Kd * (large step) / Ts, which is a massive output spike. This spike can cause audible transients, mechanical shock, and actuator wear.

The standard solution is to compute the derivative on the measurement rather than the error: D = -Kd * d(measurement)/dt instead of D = Kd * d(error)/dt. Since the measurement does not change instantaneously when the setpoint changes (the plant has inertia), the derivative remains smooth. The two formulations are mathematically equivalent for the derivative of the error when the setpoint is constant, but they differ at the instant of a setpoint change. Every professional PID implementation uses derivative on measurement.

A secondary mitigation is setpoint filtering: instead of an instantaneous step change, ramp the setpoint through a rate limiter or pass it through a first-order filter. This converts the step into a smooth transition, eliminating the kick even on the error derivative. The combination of derivative on measurement plus setpoint filtering ensures smooth actuator commands under all conditions.

**Q11: Compare complementary filter vs. Kalman filter for IMU sensor fusion. When would you choose each?**

The complementary filter combines accelerometer and gyroscope data using a fixed-weight blend: `angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle`. It is a first-order IIR filter with a single tunable parameter (alpha). It is easy to implement (5 lines of code), computationally cheap (a few multiplies and adds), and works surprisingly well for single-axis angle estimation.

The Kalman filter is a statistically optimal estimator that models the system state, process noise, and measurement noise explicitly. It maintains a state estimate and an uncertainty covariance, updating both with each new measurement. For a 1-axis angle, the state is [angle, gyro_bias], and the Kalman filter automatically estimates and removes gyro bias. It adapts its blending weights based on the relative noise levels of each sensor.

Choose the complementary filter when: you need a quick, robust solution; you are on a resource-constrained MCU; you only need 1-axis angle; and the system is well-characterized (you can pick alpha empirically). Choose the Kalman filter when: you need multi-axis estimation (6-DOF or 9-DOF with magnetometer); you need to estimate gyro bias dynamically; sensor noise characteristics change over time; or you need provably optimal estimation for a safety-critical application. For this project, the complementary filter is the right choice.

**Q12: How do you choose the PWM frequency for motor control? What happens if it is too low or too high?**

The PWM frequency must satisfy several constraints:

**Too low (below ~1 kHz)**: The motor current ripple is large because the winding inductance cannot smooth the pulsed current. This causes audible buzzing, increased I2R heating (RMS current is higher than average current due to ripple), and potentially rough low-speed operation. Below about 100 Hz, the motor may cog (stutter) because it accelerates and decelerates within each PWM cycle.

**Too high (above ~50 kHz)**: Switching losses in the MOSFET driver increase linearly with frequency. The driver transistors spend more time in their linear region during transitions, dissipating more power. Gate drive circuits may not fully turn the MOSFETs on/off, increasing on-resistance and losses. EMI increases. For the TB6612FNG, the datasheet specifies operation up to 100 kHz, but efficiency decreases above 50 kHz.

**Sweet spot (10-25 kHz)**: Above the audible range (20 kHz) eliminates buzzing. The motor inductance smooths current effectively. Switching losses are minimal. Timer resolution is adequate (5000+ steps at 100 MHz clock). 20 kHz is the industry standard for small motor drivers. For large industrial drives, frequencies of 4-16 kHz are common because switching losses in high-power IGBTs are more significant.

**Q13: Describe the discrete-time PID implementation. What happens if the sampling rate is too slow?**

The standard position-form discrete PID is:

```
e[k] = setpoint - measurement[k]
P[k] = Kp * e[k]
I[k] = I[k-1] + Ki * Ts * e[k]
D[k] = Kd * (measurement[k-1] - measurement[k]) / Ts
output[k] = P[k] + I[k] + D[k]
```

If the sampling rate is too slow relative to the plant dynamics, several problems arise: (1) Phase lag from the zero-order hold (the output is held constant between samples) reduces the phase margin, potentially destabilizing the system. A rule of thumb is to sample at 10-20x the desired closed-loop bandwidth. (2) The discrete derivative becomes a poor approximation of the continuous derivative, introducing additional phase error. (3) The system cannot react to disturbances faster than the sample rate, reducing disturbance rejection. (4) Aliasing: if the plant has dynamics faster than Nyquist (half the sample rate), they fold back into the control bandwidth and corrupt the controller.

For this motor control project: the motor's mechanical time constant is ~20-50 ms, giving a bandwidth of roughly 20-50 rad/s. A 1 kHz sample rate gives 60-300x oversampling, which is more than sufficient. At 100 Hz, you would still have 6-30x oversampling, probably adequate. At 10 Hz, the system would likely be unstable or severely underdamped.

**Q14: You observe that your PID controller oscillates with a low-frequency period (several seconds). What is the likely cause?**

Low-frequency oscillation (period much longer than the plant's natural response time) is the hallmark of integral windup or an over-aggressive integral term. The mechanism:

1. A disturbance causes an error.
2. The integral accumulates error over time.
3. By the time the system responds, the integral has accumulated a large value.
4. This causes overshoot past the setpoint.
5. Now the error is in the opposite direction, and the integral begins accumulating in reverse.
6. The cycle repeats with a period determined by how fast the integral accumulates and unwinds.

Diagnosis: temporarily set Ki to zero. If the oscillation stops, the integral term is the cause. Solutions: (1) Reduce Ki. (2) Implement anti-windup if not already present. (3) Increase Kd to damp the overshoot. (4) Check if there is a large transport delay in the system (delay makes integral control harder).

Other possible causes of low-frequency oscillation include: actuator backlash (dead zone causes a limit cycle), quantization in the sensor or actuator (creates a discrete-amplitude limit cycle), or an outer loop interacting with the inner loop in a cascaded control system.

**Q15: Explain the STM32 I2C peripheral's DMA mode. Why is it important for a 1 kHz control loop?**

The STM32 I2C peripheral can be configured to use DMA (Direct Memory Access) for both transmit and receive. In DMA mode, after the CPU initiates an I2C transaction (sending the START condition, slave address, and register address), the DMA controller handles all subsequent byte transfers between the I2C data register and a memory buffer without CPU intervention. The CPU is free to do other work and is notified via interrupt when the transfer completes.

For a 1 kHz control loop, this matters because reading 14 bytes from the MPU-6050 at 400 kHz I2C takes approximately 0.4 ms. Without DMA, the CPU is busy-waiting (polling flags or sitting in interrupt handlers) for 40% of the 1 ms loop period. With DMA, the CPU initiates the read, then immediately computes the PID output using the previous sample's data, updates the PWM, handles UART I/O, and checks for the DMA completion flag. This pipelining is crucial when the loop has multiple time-consuming I/O operations.

Configuration: enable DMA1 Stream 0 (I2C1_RX) or the appropriate stream for your I2C peripheral. Set the DMA to peripheral-to-memory mode, memory increment, 8-bit data width, and the transfer count to 14. Enable the I2C DMA request bit (I2C_CR2_DMAEN). In the DMA transfer-complete ISR, set a flag indicating new data is available and process it in the next control loop iteration.

**Q16: What is clock stretching in I2C and when does it occur?**

Clock stretching is when an I2C slave device holds the SCL line low after the master releases it, pausing the clock. The master must detect that SCL has not gone high (by reading the pin state) and wait until the slave releases it. This mechanism allows slow slave devices to pace the communication.

The MPU-6050 uses clock stretching when its internal ADC is busy or when the I2C slave controller needs additional time to load the next byte from the FIFO. The stretching duration is typically 1-50 microseconds. If the master does not support clock stretching (i.e., it bit-bangs SCL without checking if SCL actually went high), the master and slave fall out of sync, causing corrupted data or bus hangs.

The STM32 I2C hardware peripheral handles clock stretching automatically: it reads the SCL pin state after releasing it and waits for the pin to actually go high. However, software (bit-banged) I2C implementations must explicitly implement this check. On the ESP32, clock stretching support depends on the I2C driver configuration -- the default driver supports it, but some lightweight libraries do not.

**Q17: How do you convert between floating-point PID gains and the actual physical units they represent?**

Understanding the units of PID gains is critical for transferring tuning between different implementations or platforms.

For an angle controller: error is in degrees, output is normalized duty cycle (-1 to +1), and time is in seconds.

- Kp has units of [duty / degree]. If Kp = 0.05, then a 10-degree error produces a 0.5 (50%) duty cycle output.
- Ki has units of [duty / (degree * second)]. If Ki = 0.01, then a constant 10-degree error accumulates 0.1 duty per second.
- Kd has units of [duty * second / degree]. If Kd = 0.001, then an angle rate of 100 degrees/second produces a 0.1 duty output.

When you change the control loop rate, the discrete gains change because Ts appears in the integral and derivative computations. If you use the standard form (Ki_discrete = Ki_continuous * Ts for integral, Kd_discrete = Kd_continuous / Ts for derivative), then the continuous gains are rate-independent. But if your implementation uses a different convention (e.g., Ki already includes Ts), changing the loop rate requires re-scaling Ki and Kd. Always document which convention your PID implementation uses.

**Q18: Your motor control system works well on the bench but behaves erratically when connected to the actual mechanism. What do you investigate?**

Systematic investigation approach:

1. **EMI**: The mechanism's motors or actuators generate electrical noise that couples into sensor signals or the MCU. Check I2C signals with the logic analyzer for glitches. Add ferrite beads on I2C lines. Improve grounding (star ground topology). Add 100nF caps across motor terminals.

2. **Mechanical resonance**: The mechanism has a different natural frequency, inertia, and friction than the bench test. The PID gains tuned on the bench may cause oscillation in the actual system. Re-tune the gains on the real mechanism.

3. **Backlash and nonlinearities**: Gears, linkages, and loose connections introduce dead zones. The PID controller commands small corrections, but nothing happens until the accumulated output overcomes the dead zone, then the system lurches. This creates limit cycles. Solutions: add a dead-band in the controller or use a nonlinear gain near zero error.

4. **Power supply**: The mechanism draws more current than the bench test, causing voltage droops that affect the MCU or sensors. Measure the supply voltage under load. Add bulk capacitance.

5. **Structural vibration**: The mechanism vibrates at frequencies that excite the IMU's accelerometer, corrupting the angle measurement. Increase the complementary filter's alpha (trust gyro more) or add mechanical vibration damping.

6. **Thermal effects**: Components heat up during sustained operation, changing motor resistance, sensor offsets, and driver characteristics. Characterize behavior over temperature.

### Senior Level

**Q19: Design a cascaded PID control system for a DC motor with position and velocity loops. Explain the tuning order and interaction between loops.**

A cascaded (inner/outer loop) architecture uses two PID controllers: an inner velocity loop and an outer position loop. The outer loop generates a velocity setpoint based on the position error; the inner loop drives the motor to achieve that velocity.

**Architecture**:
```
Position_SP -> [Position PID] -> Velocity_SP -> [Velocity PID] -> PWM -> Motor -> Velocity -> Position
                     ^                               ^                       |          |
                     |                               |                       |          |
                     +---------- Position -----------+---------- Velocity ---+          |
                                 (from encoder)                  (from encoder          |
                                                                  or tachometer)        |
```

**Tuning order**: Always tune inner loop first, then outer loop. The inner velocity loop must be faster (higher bandwidth) than the outer position loop, typically 5-10x faster. Tune the velocity loop alone (with a velocity setpoint from UART, not from the position controller) using step response testing. Aim for fast, well-damped response with minimal overshoot. Then close the outer position loop with a low Kp and gradually increase it.

**Interaction**: If the outer loop is tuned too aggressively (bandwidth approaches the inner loop's bandwidth), the loops interact destructively and the system becomes unstable. The outer loop must respect the inner loop's bandwidth as a constraint. The inner loop also provides natural disturbance rejection: load torque disturbances are handled by the velocity loop before they significantly affect position.

**Advantage over single loop**: A single position PID must simultaneously handle both the fast dynamics (electrical time constant, current response) and slow dynamics (mechanical inertia). The cascaded approach separates these time scales, making each loop simpler to tune and more robust to disturbances.

**Q20: Explain how to implement a Kalman filter for angle estimation with the MPU-6050. What are the state vector, state transition matrix, and measurement model?**

**State vector**: `x = [angle, gyro_bias]^T`. We estimate both the angle and the gyroscope bias simultaneously. The bias is modeled as a slowly varying random walk.

**State transition (prediction)**:
```
x_predicted = F * x_previous + B * u
F = [[1, -dt],    // angle_new = angle_old - bias * dt (bias subtracts from gyro)
     [0,  1  ]]   // bias_new = bias_old (random walk)
B = [[dt],         // gyro_rate (measured) contributes to angle
     [0 ]]
u = gyro_rate_measured
```

The prediction step gives: `angle_pred = angle + (gyro_rate - bias) * dt`, `bias_pred = bias`.

**Process noise covariance Q**: Represents uncertainty in the model.
```
Q = [[Q_angle * dt,    0          ],
     [0,               Q_gyro_bias * dt]]
```
Q_angle is the accelerometer noise variance; Q_gyro_bias is the gyro bias drift rate. Typical values: Q_angle = 0.001, Q_gyro_bias = 0.003. These are tuning parameters.

**Measurement model**: The accelerometer provides an angle measurement: `z = H * x`, where `H = [1, 0]` (the accelerometer measures angle but not gyro bias). The measurement noise covariance R is a scalar representing accelerometer noise variance (R = 0.03 is a typical starting point).

**Update step** (standard Kalman equations):
```
y = z - H * x_predicted                    // Innovation
S = H * P_predicted * H^T + R              // Innovation covariance
K = P_predicted * H^T / S                  // Kalman gain (2x1 vector)
x_updated = x_predicted + K * y            // State update
P_updated = (I - K * H) * P_predicted      // Covariance update
```

The Kalman gain K automatically balances trust between the gyro (prediction) and accelerometer (measurement) based on their respective noise levels. When the system first starts, P is large (high uncertainty), so K is large (trust the accelerometer). As the filter converges, P decreases and K settles to a steady-state value. The steady-state Kalman filter is mathematically equivalent to the complementary filter with an optimal alpha derived from the noise parameters.

**Q21: How would you implement a rate-limited setpoint trajectory to avoid exciting mechanical resonances?**

Instead of commanding an instantaneous step change in the setpoint, generate a smooth trajectory that respects velocity and acceleration limits. This is called trajectory generation or motion profiling.

**Trapezoidal profile**: The setpoint moves at a constant acceleration until reaching a maximum velocity, cruises at that velocity, then decelerates at a constant rate to the target. This produces a trapezoidal velocity profile and an S-curved position profile.

```c
typedef struct {
    float target;       // Final target position
    float current;      // Current trajectory position
    float velocity;     // Current trajectory velocity
    float max_vel;      // Maximum velocity (deg/s)
    float max_accel;    // Maximum acceleration (deg/s^2)
} trajectory_t;

void trajectory_update(trajectory_t *traj, float dt) {
    float error = traj->target - traj->current;
    float stopping_distance = (traj->velocity * traj->velocity) / (2.0f * traj->max_accel);

    if (fabsf(error) < 0.01f && fabsf(traj->velocity) < 0.1f) {
        // At target, stop
        traj->current = traj->target;
        traj->velocity = 0.0f;
        return;
    }

    float desired_accel;
    if (fabsf(error) > stopping_distance) {
        // Accelerate toward target
        desired_accel = (error > 0) ? traj->max_accel : -traj->max_accel;
    } else {
        // Decelerate to stop at target
        desired_accel = -(traj->velocity * traj->velocity) / (2.0f * error);
    }

    traj->velocity += desired_accel * dt;

    // Clamp velocity
    if (traj->velocity > traj->max_vel) traj->velocity = traj->max_vel;
    if (traj->velocity < -traj->max_vel) traj->velocity = -traj->max_vel;

    traj->current += traj->velocity * dt;
}
```

The PID controller then tracks `traj->current` (the smooth trajectory) rather than the raw target. This decouples the trajectory planning from the feedback control, each handling a different concern: the trajectory planner ensures physically feasible motion, while the PID controller handles tracking errors and disturbance rejection.

For avoiding resonances specifically: choose max_accel such that the jerk (derivative of acceleration) at the transition points does not excite the mechanism's resonant frequency. An S-curve profile (with limited jerk) is smoother than a trapezoidal profile and better at avoiding resonance excitation.

**Q22: Discuss the tradeoffs between using a hardware timer interrupt vs. a real-time operating system (RTOS) task for the control loop.**

**Bare-metal timer interrupt**: A hardware timer (e.g., TIM5) generates an interrupt at 1 kHz. The ISR sets a flag or directly executes the control computation. Advantages: deterministic timing (jitter is sub-microsecond if no higher-priority interrupts preempt), minimal overhead (no context switch), simple (no RTOS complexity). Disadvantages: the control code runs at interrupt priority, blocking other interrupts if it takes too long. I2C reads (which may involve waiting for the peripheral) should not be done in the ISR. You need a careful split: ISR triggers the computation, main loop handles I/O.

**RTOS task** (e.g., FreeRTOS): The control loop runs in a high-priority task that wakes every 1 ms using `vTaskDelayUntil()`. Advantages: the control task can call blocking I/O functions (HAL_I2C_Mem_Read) because the RTOS scheduler can preempt it if a higher-priority task needs the CPU. Multiple tasks can coexist (control task, UART task, display task) with clean separation. Disadvantages: RTOS adds RAM overhead (each task needs its own stack, typically 256-1024 bytes), introduces context-switch latency (2-5 us on Cortex-M4), and the timing jitter is higher than a hardware interrupt (depends on what other tasks are doing when the timer expires).

**Recommendation**: For a single-axis PID with one sensor and one actuator, bare-metal is simpler and sufficient. For a complex system with multiple control loops, communication interfaces, and a display, an RTOS provides better code organization. On the STM32F411RE with 128 KB RAM, FreeRTOS fits comfortably.

The hybrid approach is common: use a timer interrupt to trigger the time-critical I/O (start DMA read from IMU, update PWM), and use the main loop or an RTOS task for the PID computation and non-critical tasks (UART, display). This gives interrupt-level timing precision for the sampling while keeping the main computation flexible.

**Q23: How would you implement auto-tuning using the relay feedback method?**

The relay feedback method (Astrom-Hagglund) is a practical alternative to Ziegler-Nichols that does not require the system to reach full oscillation. A relay (bang-bang controller) is placed in the feedback loop: the output is +d when the error is positive and -d when the error is negative, where d is a fixed relay amplitude (e.g., 20% duty cycle).

The relay forces the system into a stable limit cycle (oscillation). The oscillation amplitude (a) and period (Tu) are measured from the sensor output. The equivalent critical gain is computed as Ku = 4d / (pi * a), and Tu is the oscillation period. These values are then used in the Ziegler-Nichols or Tyreus-Luyben table to compute PID gains.

```c
void relay_autotune(void) {
    float d = 0.2f;        // Relay amplitude (20% duty)
    float setpoint = 0.0f;
    float peaks[10], periods[10];
    int peak_count = 0;

    // Run relay for several oscillation cycles
    uint32_t start = HAL_GetTick();
    float prev_error = 0.0f;
    float prev_measurement = 0.0f;
    uint32_t last_crossing = start;

    while (peak_count < 6 && (HAL_GetTick() - start) < 10000) {
        float measurement = get_filtered_angle();
        float error = setpoint - measurement;

        // Relay output
        float output = (error > 0) ? d : -d;
        motor_set(output);

        // Detect zero crossings for period measurement
        if ((prev_error > 0 && error <= 0) || (prev_error < 0 && error >= 0)) {
            uint32_t now = HAL_GetTick();
            if (peak_count > 0) {
                periods[peak_count - 1] = (now - last_crossing) * 2.0f;  // Full period = 2 half-periods
            }
            last_crossing = now;
            peak_count++;
        }

        // Track peak amplitude
        // (record max |measurement - setpoint| between zero crossings)

        prev_error = error;
        HAL_Delay(1);
    }

    // Compute averages
    float Tu_avg = average(periods, peak_count - 1) / 1000.0f;  // ms to seconds
    float a_avg = average(peaks, peak_count);
    float Ku = 4.0f * d / (3.14159f * a_avg);

    // Ziegler-Nichols PID
    pid.Kp = 0.6f * Ku;
    pid.Ki = 1.2f * Ku / Tu_avg;
    pid.Kd = 0.075f * Ku * Tu_avg;
}
```

Advantages over manual Z-N: (1) The oscillation amplitude is bounded by the relay amplitude d, which you control. (2) The method works automatically without operator skill. (3) It converges in a few oscillation cycles (5-10 seconds for a fast motor system).

Disadvantages: (1) The Z-N gains from relay feedback still need manual refinement. (2) The method assumes a linear plant; nonlinearities (friction, backlash) cause asymmetric oscillation. (3) For higher-order plants, the relay may not excite the critical frequency accurately. (4) Adding hysteresis to the relay (switching at +/- h instead of 0) can improve robustness but complicates the Ku calculation: Ku = 4d * sqrt(d^2 - h^2) / (pi * a * d).

**Q24: Explain how quantization noise in the PWM duty cycle affects control performance. How many bits of resolution are needed?**

The PWM duty cycle is quantized to discrete steps (ARR + 1 steps). This quantization acts as a noise source in the control loop. If the PID controller computes a duty cycle of 50.03% but the nearest available step is 50.00% or 50.02%, the difference is a quantization error that creates a limit cycle: the controller alternates between two adjacent duty steps, never settling exactly at the desired output.

The amplitude of the limit cycle is determined by the quantization step size. With ARR = 4999 (5000 steps), the step size is 0.02% of full scale. For a motor spinning at 5000 RPM at full duty, one LSB corresponds to 1 RPM. If the position controller needs to hold position to within 0.1 degrees, and the motor gain is 100 RPM/degree, then 1 RPM of velocity uncertainty corresponds to 0.01 degrees of position uncertainty -- well within the requirement.

**Rule of thumb**: You need enough PWM resolution that one LSB of duty cycle produces a force/torque smaller than the disturbances the controller must reject. For most hobby-scale motor control, 10-12 bits (1024-4096 steps) is sufficient. Industrial servo drives use 14-16 bits. For this project, 5000 steps (12.3 bits) is more than adequate.

If you need more resolution at a given PWM frequency, you can use a higher timer clock (up to 100 MHz on the F411RE) or use dithering: alternate between two adjacent duty levels across multiple PWM periods to achieve fractional-step average duty. This technique trades temporal resolution for amplitude resolution.

**Q25: A customer reports that the motor control system works perfectly at room temperature but oscillates in a cold environment. What is happening?**

Several temperature-dependent mechanisms can cause this:

1. **Motor resistance**: DC motor winding resistance increases slightly with temperature (copper has a positive temperature coefficient: +0.4%/C). In cold conditions, resistance is lower, so the motor draws more current for the same voltage, increasing the motor gain (torque per volt). If the PID was tuned at room temperature, the higher gain in cold conditions pushes the loop gain above the stability margin, causing oscillation. Solution: reduce Kp by the expected gain change, or implement gain scheduling based on a temperature measurement.

2. **Lubricant viscosity**: Grease and oil become more viscous in the cold, increasing friction. This can cause stick-slip behavior (stiction): the motor stalls at low effort, then lurches when the effort overcomes static friction. This creates a limit cycle. The PID integral term accumulates during the stall, then releases a burst of energy. Solution: add a deadband or use a friction compensator (feedforward term that overcomes static friction).

3. **Sensor drift**: The MPU-6050's gyroscope offset drifts with temperature (the datasheet specifies +/- 40 deg/s full-scale over temperature). If the gyro bias calibration was done at room temperature and the operating temperature is significantly different, the bias is wrong, causing the angle estimate to drift, which the PID tries to correct, creating oscillation about a moving target. Solution: re-calibrate at operating temperature, or use the MPU-6050's temperature sensor to apply a temperature compensation curve.

4. **Capacitor ESR**: Electrolytic capacitor ESR increases significantly at low temperatures, reducing decoupling effectiveness. Motor switching noise couples into the sensor signals, corrupting the PID input. Solution: use ceramic capacitors (lower ESR temperature sensitivity) or add additional decoupling.

The diagnostic approach: capture step response data at both temperatures and compare. If the oscillation frequency matches the PID loop's natural frequency, it is a gain margin issue. If the oscillation is a low-frequency limit cycle, it is likely friction-related. If the oscillation is random/irregular, it is likely noise-related.

**Q26: How would you implement a feedforward term to improve disturbance rejection?**

Feedforward control uses knowledge of the disturbance or the desired trajectory to generate an output that anticipates the required control effort, without waiting for the error to develop. This dramatically improves transient response.

For a DC motor position controller, the feedforward term can include:

1. **Velocity feedforward**: If you know the desired velocity from the trajectory generator (Step 21), multiply it by the inverse of the motor's velocity constant (Kv) to get the approximate duty cycle needed: `ff_velocity = desired_velocity / Kv`.

2. **Acceleration feedforward**: Multiply the desired acceleration by the motor's inertia divided by the torque constant: `ff_accel = desired_accel * J / Kt`. This accounts for the torque needed to accelerate the load.

3. **Friction compensation**: A constant feedforward term that overcomes Coulomb friction: `ff_friction = sign(desired_velocity) * friction_duty`. This prevents the integral term from having to build up slowly to overcome friction.

```c
float output = pid_update(&pid, setpoint, measurement, dt)
             + Kff_v * desired_velocity
             + Kff_a * desired_acceleration
             + Kff_f * sign(desired_velocity);
```

The PID controller then only needs to correct for modeling errors and unmeasured disturbances, which are much smaller than the nominal trajectory forces. The feedforward terms can be determined experimentally: command different velocities and accelerations open-loop, measure the required duty cycle, and fit a linear model.

**Q27: Explain the electrical noise considerations when routing I2C signals near a motor driver on a breadboard.**

Motor drivers generate several types of electrical noise:

1. **Conducted noise**: High di/dt switching transients travel through shared power and ground traces. If the motor driver and the IMU share the same ground wire/rail, the voltage drop across that wire (V = L * di/dt) appears as a common-mode disturbance on the sensor signals. On a breadboard, the ground rails have significant inductance (1-5 nH per cm) and resistance.

2. **Radiated noise**: The motor leads and the traces between the driver and motor act as antennas, radiating electromagnetic energy at the switching frequency and its harmonics. I2C signal lines act as receiving antennas. The coupling increases when the signal lines run parallel to and close to the motor power lines.

3. **Motor brush noise**: DC motor brushes create arcing as they make and break contact with the commutator. This generates broadband noise (MHz range) that couples through the motor leads and radiates.

**Mitigation on a breadboard**:
- Keep motor power wiring on one side of the breadboard and sensor/I2C wiring on the other side.
- Use a separate ground wire for the motor circuit and the sensor circuit, tied together at one point only (star ground).
- Place 100nF ceramic capacitors directly across the motor terminals (solder them to the motor, not on the breadboard).
- Keep I2C wires short (under 10 cm) and twist the SDA/SCL pair together with a ground wire.
- If problems persist, add a small ferrite bead (100 ohm at 100 MHz) in series with each I2C line.
- Use the MPU-6050's digital low-pass filter (DLPF) at a moderate setting (config register 0x1A) to filter high-frequency noise from the sensor output before it reaches your software.

**Q28: Compare implementing this project on an STM32F411RE vs. an ESP32. What are the major tradeoffs?**

**STM32F411RE (Cortex-M4F)**:
- Deterministic timing: hardware timers with sub-microsecond jitter, no background OS tasks.
- True hardware I2C peripheral with DMA support.
- 100 MHz single-core, FPU, predictable instruction timing.
- Bare-metal or lightweight RTOS, full control over memory and peripherals.
- Professional ecosystem: STM32CubeMX, CubeIDE, ST-LINK debugger with breakpoints and watchpoints.
- No WiFi/Bluetooth (unless you add an external module).
- Better for learning embedded systems fundamentals because you must understand the hardware.

**ESP32 (Xtensa LX6 dual-core, 240 MHz)**:
- Built-in WiFi and Bluetooth for wireless tuning interface and data logging.
- More RAM (520 KB SRAM + 4 MB PSRAM typically) and flash (4-16 MB).
- FreeRTOS is mandatory (the ESP-IDF is built on it), which adds complexity but provides multi-tasking.
- Less deterministic: WiFi/BT stack runs on core 0 and can cause scheduling jitter on core 1. I2C is handled by software (not a true hardware peripheral), and the FreeRTOS tick interrupt adds jitter.
- Timer resolution: LEDC PWM peripheral is configurable but less flexible than STM32 timers. MCPWM peripheral is more suitable for motor control.
- Arduino framework available, which is convenient but hides important details.
- Harder to debug (no SWD/JTAG by default on most dev boards; printf debugging is the norm).

**Recommendation**: For learning embedded control systems, use the STM32F411RE. It forces you to understand clocks, timers, DMA, and interrupts at the register level. For a product prototype that needs wireless connectivity, the ESP32 is more practical. For a competition or quick demo, the ESP32 with Arduino is faster to get running but teaches less about the underlying hardware.

**Q29: How do you verify that your control loop is meeting its real-time deadline, and what do you do if it is not?**

**Measurement**: Toggle a GPIO pin at the start and end of each control loop iteration. Measure the pulse width with the logic analyzer. This gives you the execution time per iteration. Also measure the period between rising edges to verify the loop rate is consistent (low jitter).

```c
// In 1 kHz control loop:
GPIOC->BSRR = GPIO_PIN_13;  // Set pin high (start of loop)

read_imu();
compute_complementary_filter();
pid_output = pid_update(&pid, setpoint, angle, dt);
motor_set(pid_output);

GPIOC->BSRR = GPIO_PIN_13 << 16;  // Set pin low (end of loop)
```

Alternatively, use a hardware timer for software profiling:

```c
uint32_t t_start = TIM2->CNT;
// ... control loop code ...
uint32_t t_elapsed = TIM2->CNT - t_start;  // Microseconds (if TIM2 at 1 MHz)
if (t_elapsed > worst_case) worst_case = t_elapsed;
```

**If the loop exceeds the deadline**:
1. Profile each section (IMU read, filter, PID, motor set, UART) to find the bottleneck.
2. Use DMA for I2C reads (saves ~400 us at 400 kHz).
3. Use DMA for UART TX (saves time proportional to data volume).
4. Reduce OLED update rate or move it to a separate low-priority context.
5. Optimize the PID computation (usually negligible, <10 us on Cortex-M4F).
6. Reduce the control loop rate (from 1 kHz to 500 Hz) if the plant dynamics allow it.
7. If using floating-point heavily, verify the FPU is enabled (check CPACR register; without FPU, float operations take 20-100x longer via software emulation).

**Jitter specification**: For a 1 kHz control loop, jitter should be under 5% (50 us). The discrete PID uses `dt` as a constant; if the actual period varies by more than 5%, the integral and derivative terms accumulate errors. If jitter exceeds this, use the actual measured dt (from a hardware timer) in the PID computation instead of a fixed constant.

**Q30: Design a safety system for a motor controller that handles sensor failure, communication loss, and actuator faults.**

A production motor controller must handle faults gracefully. Key failure modes and responses:

**Sensor failure (IMU stops responding)**:
- Detection: I2C read returns HAL_ERROR or HAL_TIMEOUT. Count consecutive failures; if > 3, declare sensor fault.
- Response: Immediately set motor output to zero (brake mode). Do not attempt to continue with stale data -- the PID controller will diverge. Set a fault LED/flag. Attempt I2C bus recovery (toggle SCL 9 times). If recovery fails, require a manual reset.

```c
if (HAL_I2C_Mem_Read(...) != HAL_OK) {
    sensor_fail_count++;
    if (sensor_fail_count > 3) {
        motor_set(0.0f);  // Brake
        fault_state = FAULT_SENSOR;
        return;
    }
} else {
    sensor_fail_count = 0;
}
```

**Communication loss (UART CLI stops responding)**:
- Less critical since UART is for tuning, not control. But implement a watchdog: if no valid command received in 60 seconds, optionally revert to default safe gains.
- More important: if the UART is the command source for the setpoint in a remote-control scenario, implement a timeout. If no setpoint update in 500 ms, ramp the motor to zero.

**Actuator fault (motor stall, overcurrent)**:
- Detection: Monitor the motor current via an ADC on a shunt resistor, or monitor the PID output -- if the output is saturated (100% duty) for > 2 seconds but the measurement is not changing, the motor is likely stalled.
- Response: Reduce duty to zero to prevent thermal damage to the motor and driver. Set a fault flag. Require manual reset after investigation.

**Watchdog timer**: Configure the STM32's Independent Watchdog (IWDG) to reset the MCU if the main loop hangs for more than 100 ms. Kick the watchdog at the end of each successful control loop iteration. This catches firmware lockups, stack overflows, and hard faults.

```c
// At startup:
IWDG->KR = 0x5555;         // Enable register access
IWDG->PR = IWDG_PR_PR_2;   // Prescaler /64
IWDG->RLR = 50;             // ~100ms timeout at 32 kHz LSI
IWDG->KR = 0xCCCC;         // Start watchdog

// In main loop (after successful PID iteration):
IWDG->KR = 0xAAAA;          // Kick watchdog
```

**Fault state machine**: Define states (NORMAL, FAULT_SENSOR, FAULT_MOTOR, FAULT_COMMS) and only allow transition out of a fault state via an explicit reset command. Never automatically resume from a fault -- the operator must verify the system is safe first.
