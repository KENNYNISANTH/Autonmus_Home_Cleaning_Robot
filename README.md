# Obstacle-Avoiding Autonomous Robot

## About the Bot

This project implements an autonomous robot capable of detecting and avoiding obstacles using ultrasonic and IR sensors, as well as providing manual control via serial commands. The robot uses DC motors for locomotion and an Electronic Speed Controller (ESC) for speed control of an auxiliary device (such as a vacuum or brush motor).

### Key Features

* **Autonomous Navigation**: Uses three ultrasonic sensors (left, front, right) and one IR sensor to detect obstacles and decide movement.
* **Manual Override**: Accepts serial commands (`F`, `B`, `L`, `R`, etc.) from Bluetooth or USB to manually control the robot.
* **Variable Speed Control**: A potentiometer connected to an analog pin sets ESC throttle from 1000µs (min) to 2000µs (max).
* **ESC Integration**: Controls an ESC via Servo library for auxiliary motor control.

## Hardware Requirements

| Component                    | Quantity | Notes                          |
| ---------------------------- | -------- | ------------------------------ |
| Arduino Uno (or compatible)  | 1        | Main controller                |
| HC-SR04 Ultrasonic Sensor    | 3        | Mounted left, front, and right |
| IR Obstacle Detection Sensor | 1        | Digital output                 |
| Servo / ESC                  | 1        | Controlled via Servo library   |
| DC Motors + Driver Circuit   | 2        | For left and right wheels      |
| Potentiometer (10kΩ)         | 1        | Speed control via analog input |
| Jumper Wires & Breadboard    | —        | For prototyping                |
| Power Supply / Battery Pack  | —        | 7–12V for Arduino and motors   |

### Pin Connections

#### Ultrasonic Sensors

* **Left Sensor**: `Trig` → D3, `Echo` → D5
* **Front Sensor**: `Trig` → D6, `Echo` → D9
* **Right Sensor**: `Trig` → D10, `Echo` → D11

#### IR Sensor

* **Output** → D2

#### Motor Driver (e.g., L298N)

* **IN1** → D4
* **IN2** → D7
* **IN3** → D8
* **IN4** → D12

#### ESC / Servo

* **Signal** → D13
* **Power & GND** → Shared with Arduino

#### Potentiometer

* **Middle Pin** → A0
* **Ends** → 5V and GND

## Software Setup

1. **Arduino IDE**: Install the latest Arduino IDE from [arduino.cc](https://www.arduino.cc).
2. **Library**: The Servo library comes pre-installed. No additional libraries needed.
3. **Sketch**: Copy the provided `ObstacleAvoidingBot.ino` code into a new sketch.
4. **Upload**: Connect the Arduino via USB, select the correct board and port, and click **Upload**.

## Code Explanation

```cpp
#include <Servo.h>
Servo esc;  // Create servo object to control ESC
```

* **Servo esc**: Controls ESC throttle signal.

```cpp
const int trigPin1 = 3, echoPin1 = 5;
const int trigPin2 = 6, echoPin2 = 9;
const int trigPin3 = 10, echoPin3 = 11;
int irpin = 2;
```

* **Trig/Echo pins**: Connected to HC-SR04 sensors.
* **irpin**: IR output pin.

```cpp
long duration1, duration2, duration3;
int distanceleft, distancefront, distanceright;
int a = 0;    // IR detection flag
char cmnd;    // Serial command
```

* **distance\***: Calculated in cm from echo duration.

```cpp
void setup() {
  pinMode(...);
  Serial.begin(9600);
  esc.attach(13);
  esc.writeMicroseconds(1000);
  delay(2000);
}
```

* **ESC arming**: Sends minimum throttle (1000µs) and waits 2 seconds.

```cpp
void loop() {
  // Read distances
  distanceleft   = measureDistance(trigPin1, echoPin1);
  distancefront  = measureDistance(trigPin2, echoPin2);
  distanceright  = measureDistance(trigPin3, echoPin3);

  int s = digitalRead(irpin);

  int val = map(analogRead(A0), 0, 1023, 1000, 2000);
  esc.writeMicroseconds(val);

  // Autonomous logic based on sensor readings...

  // Manual override via Serial commands
  if (Serial.available()) {
    cmnd = Serial.read();
    handleSerial(cmnd);
  }
}
```

* **measureDistance()**: Triggers ultrasonic and reads echo duration.
* **map()**: Scales potentiometer reading to ESC signal range.
* **Serial control**: Reads a character and moves accordingly.

## Usage

1. **Power Up**: Supply power to Arduino and motors.
2. **Calibrate ESC**: Ensure ESC arming beep sequence completes.
3. **Autonomous Mode**: Robot will navigate avoiding obstacles.
4. **Manual Mode**: Send characters via Serial Monitor or Bluetooth:

   * `F` → Forward
   * `B` → Backward
   * `L` → Turn Left
   * `R` → Turn Right
   * `I`, `J`, `G`, `H` → Diagonal movements

## Future Improvements

* Add PID control for smoother turns.
* Integrate a Bluetooth module (HC-05) for wireless control.
* Implement path planning (SLAM) for mapping.

