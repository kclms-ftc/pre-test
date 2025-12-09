# Hardware Requirements for FTC DECODE 2025-2026 Programming

To successfully program a robot for the DECODE game and fully utilize the features described in the development guide (Autonomous pathing, Vision processing, Sensor-based scoring), the following hardware is required or highly recommended.

## 1. Control System & Electronics
The core "brain" and power system required to run the code.
*   **REV Control Hub:** The primary robot controller.
    *   *Requirement:* Must be running Control Hub OS 1.1.2 or later.
    *   *Requirement:* Firmware 1.8.2 or later.
*   **REV Expansion Hub (Optional):** Required if your design uses more than 4 motors or 6 servos (the guide budgets for 8 motors and 10 servos).
*   **Battery:** 12V NiMH Battery (REV or goBILDA) with a main power switch (20A fuse).

## 2. Sensors (Critical for Software Automation)
The "Winning Robot Profile" and "Software Development" phases heavily rely on these sensors to enable Autonomous features and automated TeleOp assists.

*   **Webcam (Vision):**
    *   *Purpose:* **Essential** for reading the OBELISK MOTIF (AprilTag IDs 21, 22, 23) in Autonomous to determine the scoring pattern. Also used for aligning to the GOAL.
    *   *Compatible Models:* Logitech C270, C920, or equivalent UVC webcams.
*   **Color Sensors (x2):**
    *   *Purpose:* Identifying if an ARTIFACT is Purple or Green inside the robot to automate sorting/launching logic (e.g., Variable `detectBallColor()` function).
    *   *Compatible Models:* REV Color Sensor V3, AndyMark Color Sensor.
*   **IMU (Inertial Measurement Unit):**
    *   *Purpose:* **Critical** for "Field-Centric" driving and precise turning (`turnToHeading` function) in Autonomous.
    *   *Note:* The Control Hub has a built-in IMU (BNO055 or BHI260AP), which is usually sufficient.
*   **Distance Sensor:**
    *   *Purpose:* Detecting distance to the GOAL or walls for consistent shooting range and positioning.
    *   *Compatible Models:* REV 2m Distance Sensor.
*   **Proximity/Touch Sensors:**
    *   *Purpose:* "Limit switches" to detect when a ball is fully seated in the launcher or to limit arm movement.

## 3. Actuators (Motors & Servos)
The sample code structures imply specific actuator capabilities:

*   **Drivetrain Motors (x4):**
    *   *Configuration:* **Mecanum Drive** is the recommended setup.
    *   *Encoders:* All drivetrain motors **MUST** have working encoders (4-wire cables) connected. The code relies on encoder ticks for `driveForward` and PID velocity control.
*   **Launcher Motors (x2):**
    *   *Requirement:* High RPM motors (e.g., goBILDA 5203) with encoders to allow the software to maintain constant flywheel speed (`runFlywheelControl` with velocity ramping).
*   **Intake Motor (x1):** Continuous rotation for collection.
*   **Servos (Various):**
    *   GATE Mechanism: Needs servos to push/hold the gate open.
    *   Launcher Hood: Precision servos to adjust the angle for "Classified" vs "Overflow" shots.

## 4. Driver Station & Human Interface
*   **Driver Station Device:**
    *   Calculated based on SDK: Android Smartphone or REV Driver Hub.
*   **Gamepads (x2):**
    *   Logitech F310 or Sony DualShock 4.
    *   *Why:* The guide specifies separate controls for Driver 1 (Chassis) and Driver 2 (Scoring systems).

## 5. Optional / Advanced Hardware
*   **SparkFun OTOS / GoBilda Pinpoint:** Supported in the new SDK 11.0 for very high-precision Optical Odometry (tracking position on the field without relying solely on wheel slip).
*   **Limelight 3A:** Supported in SDK 11.0 for advanced AI vision tracking.
