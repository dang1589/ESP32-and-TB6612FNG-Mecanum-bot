// Credits

This project uses the following libraries:
- <TB6612_ESP32.h>     by https://github.com/pablopeza/TB6612FNG_ESP32
- <Bluepad32.h>        by https://github.com/ricardoquesada/bluepad32
- <Adafruit_GFX.h>     by https://github.com/adafruit/Adafruit-GFX-Library
- <Adafruit_SSD1306.h> by https://github.com/adafruit/Adafruit_SSD1306
  
  **AI tools were used for debugging and code review**

// Component list

- ESP32 DEVKIT V1
- TB6612FNG x2
- 12V motors x4 (Testing using JGA25-370 at 280rpm)
- Power supply of maximum 15V (Testing using 3x18650 3.7V)
- 0.96 inches OLED display (optional)
- Buck converter

Wiring is based on the provided schematic.


// What this can do

- Control using a Bluetooth controller via Bluepad32 (Testing using Gamesir Nova 2 Lite).
- Variable speed based on controller's XY input.
- Full 360 deg straffle using mecanum control algorithm (src: https://youtu.be/gnSW2QpkGXQ?si=y35pBZLBLJ-lp8x8).
- Forward/Backward can be inverted using a button on the controller.


// How to use the variables

There are 5 variables that you need to care about
MAX_WHEEL_SPEED; ACCEL_RATE; DEADZONE; SNAP_ANGLE; JOY_MAX;

- MAX_WHEEL_SPEED : From 1 to 255, aka the PWM values that you want.
- ACCEL_RATE      : The rate at which the wheel spins up/ down. The lower the value, the longer it takes to speed up.
- DEADZONE        : Prevent stickdrift in controllers.
- SNAP_ANGLE      : Provide assistance for driving in 4 main directions (Forward, Backward, Strafle Left, Strafle Right).
- JOY_MAX         : Joystick max value, usually 512.






