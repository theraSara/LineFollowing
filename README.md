# Autonomous Robot Control and Lane Detection

## Overview
This project consists of two main components:
1. **Arduino-based autonomous robot control** with obstacle detection and PID control.
2. **MATLAB-based lane detection** in a video feed for mid-lane position estimation.

---

## Component 1: Arduino-Based Robot Control

### Features
- **Modes of Operation**:
  - `LineFollow`: Robot follows a line using sensor data and PID control.
  - `FollowMe`: Robot adjusts movement based on distance from obstacles detected by an ultrasonic sensor.
- **PID Control**:
  - Two separate PID controllers (for each mode) optimize motor speed and direction based on sensor inputs.
- **Ultrasonic Distance Sensor**:
  - Measures distances to switch between `LineFollow` and `FollowMe` modes.
- **Motor Control**:
  - Dynamically adjusts motor speed and direction based on PID output.

### Key Functions
- **`AutomaticDrive`**: Controls the robot's motors based on PID output.
- **`readUltrasonicDistance`**: Reads distance using an ultrasonic sensor.
- **`getPosition`**: Computes the robot's position relative to the line.
- **`displayMeter`**: Displays the robot's position visually on the serial monitor.
- **`displayRawSensor`**: Shows raw sensor readings for debugging.

### Setup and Configuration
1. **Connect Motors and Sensors**:
   - Motors are connected to pins `10`, `11` (right) and `5`, `6` (left).
   - Ultrasonic sensor is connected to appropriate trigger and echo pins.
   - Ensure the line-following sensor is connected to I2C (address `0x3E`).
2. **PID Configuration**:
   - PID tunings can be adjusted in the code using `Kp`, `Ki`, and `Kd`.

### Operation
- Robot starts in `LineFollow` mode.
- Switches to `FollowMe` mode when the ultrasonic sensor detects an obstacle closer than `ObstacleDistance`.
- Returns to `LineFollow` mode once the obstacle is cleared.

---

## Component 2: MATLAB Lane Detection

### Features
- **Lane Detection**:
  - Detects left and right lane boundaries using edge detection and the Hough Transform.
- **Mid-Lane Estimation**:
  - Computes the midpoint between left and right lanes to identify the vehicle's position in the lane.
- **Visualization**:
  - Displays detected lane boundaries and mid-lane position on video frames.

### Key Steps
1. **Preprocessing**:
   - Convert each frame to grayscale.
   - Apply minimum and maximum filters to enhance edges.
   - Binarize the edge-detected image.
2. **Hough Transform**:
   - Detect lines in the binarized image.
   - Filter lines by angle to classify as left or right lane boundaries.
3. **Mid-Lane Calculation**:
   - Compute midpoints of the left and right lanes.
   - Average these midpoints to determine the mid-lane position.

### Usage
1. Place your video file in the same directory as the script and name it `My Movie 7.mp4`.
2. Run the MATLAB script to process the video and display lane detection results.
3. Use the `SimpleLaneDetection` function to process individual frames for debugging.

### Example Output
- Detected lanes are overlaid in green (left) and blue (right).
- Mid-lane position is marked with a white `X`.

---

## Improvements
### Arduino
- Add error handling for missing sensor data in `LineFollow` mode.
- Optimize PID parameters for smoother transitions and better real-world performance.

### MATLAB
- Improve robustness of mid-lane estimation when one lane boundary is missing.
- Use adaptive thresholds for edge binarization to handle varying lighting conditions.

---

## Requirements
### Arduino
- Arduino Uno or compatible microcontroller.
- Motors, motor drivers, ultrasonic sensor, and line-following sensor.
- `PID_v1` and `CircularBuffer` libraries.

### MATLAB
- MATLAB with the Image Processing Toolbox.

