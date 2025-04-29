# Changelog

All notable changes to this project are documented here.

---

## [v1.0.1] - 29/04/25
### Added
- Reintroduced and completed missing inline comments in `main.c` to improve clarity and maintainability of all major logic blocks.

---

## [v1.0.0] - 29/04/25
### Added
- Final integration of all modules: ADC input, dual servo PWM output, real-time waveform capture, and full display output.
- Finished implementation of `displayWaveforms()` including frequency, duty cycle, and angle output per channel.
- Implemented `printNumberSHORT()` to reduce flicker and optimize display updates for numeric fields.
- Polished startup behavior with `servo_startup_ramp_stepper()` to center servos smoothly on boot.
- Ensured SPI bus stability using `waitForSPIReady()` after each display command.

### Changed
- General cleanup and optimization across display rendering, sampling, and timing logic.
- Removed redundant delays; replaced with SPI transfer status checks to improve frame stability.
- Finalized all GPIO mapping decisions (e.g., PB4 as D/C for display).

---

## [v0.4.0] - 28/04/25
### Added
- Introduced `ACTIVE_SPI` abstraction to decouple display driver from hardcoded `SPI1`.
- Added real-time angle calculation based on PWM duty (0–180° mapping).
- Added rising/falling edge markers and enhanced waveform timing visuals.
- Used `printText` to label waveform zones as "Servo X" and "Servo Y".
- Added startup UI splash with "Initializing..." and display clearing logic.

### Changed
- Changed second ADC input from **PA1 (CH6)** to **PB0 (CH15)** for layout and pin flexibility.
- Updated ADC sampling configuration to support the new channel.
- Scaled waveform width dynamically based on scroll mode.
- Improved redraw efficiency by avoiding unnecessary re-renders and reducing SPI overhead.
- Separated waveform areas into scrolling left (Servo X) and right (Servo Y) halves.

---

## [v0.3.0] - 27/04/25
### Added
- Enabled TIM2 input capture on PA5 and PB3 to measure PWM period and pulse width.
- Calculated live frequency and duty cycle of servo input signals.
- Implemented EXTI9_5 interrupt for PB5 button to toggle scroll mode.
- Displayed waveform data (green = HIGH, red = LOW, white = edge) for each servo.
- Added initial angle/duty text display at bottom of the screen.

### Changed
- Moved ST7735 display D/C pin from PA5 to PB4 to avoid GPIO conflict with TIM2 input.
- Moved display commands to use PB4 correctly in `DCLow()` and `DCHigh()`.
- Enhanced layout with left/right panes and structured bottom bar for text output.
- Refined deadzone scaling and smoothing behavior to improve stability.

---

## [v0.2.0] - 26/04/25
### Added
- Implemented ADC with dual-channel sequential conversion (PA0 = CH5, PA1 = CH6).
- Created rolling average buffer (8-sample) for smoothing noisy analog inputs.
- Developed dynamic deadzone logic that responds to rate-of-change in potentiometer readings.
- Mapped potentiometer input to servo PWM via `pot_to_servo()`.
- Set up TIM1 PWM output for servo control on PA8 and PA9.
- Configured USART2 with redirected `printf` for real-time debugging.
- Integrated existing ST7735 display functions (e.g., `fillRectangle`, `putPixel`, `printText`) for screen output.

### Changed
- Tuned ADC sample time and calibration sequence for accurate readings.
- Adjusted PWM scaling to match servo pulse width requirements.
- Refactored system clock setup for 80MHz using PLL and configured SysTick for 1ms timing.
