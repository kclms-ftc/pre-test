# DECODE Robot Code - TeamCode

This folder contains the complete software suite for the FTC DECODE game robot. It is designed for a **Mecanum Drive** robot using the **REV Control System**, as laid out in the `ftc_decode_guide.md`.

## Structure

*   **`hardware/RobotHardware.java`**: The central hardware configuration class. It abstracts the motors, servos, and sensors.
    *   **Abstraction**: Allows you to change motor config in one place without breaking Auto/TeleOp.
    *   **Hardware Map**:
        *   `left_front_drive`, `right_front_drive`... (Drivetrain)
        *   `launcher_left` (Launcher)
        *   `intake_motor` (Intake)
        *   `gate_servo`, `angle_servo` (Servos)
        *   `Webcam 1` (Vision)
*   **`teleop/DecodeTeleOp.java`**: The main driver control program.
    *   **Driver 1**: Controls the chassis using Field-Centric Mecanum driving (Left stick moves relative to field, Right turn).
    *   **Driver 2**: Controls Mechanisms.
        *   `A`/`B`: Launcher Speed (Classified/Overflow).
        *   `Triggers`: Intake/Outtake.
        *   `D-Pad`: Gate Control.
*   **`autonomous/DecodeAuto.java`**: The Autonomous routine.
    *   **Vision**: Uses `VisionPortal` and `AprilTagProcessor` to detect tags 21, 22, 23 (GPP, PGP, PPG).
    *   **Drive**: Uses Encoder-based `driveStraight` logic.

## Setup Instructions

1.  **Configure Robot Controller**:
    *   Go to the Driver Station App > Configure Robot.
    *   Use the names specified in `RobotHardware.java` (e.g., "left_front_drive").
2.  **Calibration**:
    *   In `RobotHardware.java`, adjust `COUNTS_PER_INCH` if your wheels are different (default 96mm/3.78").
    *   Adjust `LAUNCHER_SPEED` constants based on testing.
3.  **Hardware Adjustment**:
    *   If using **2-Motor Tank Drive** (Standard Starter Kit):
        *   Update `RobotHardware.java` to only have `leftDrive` and `rightDrive`.
        *   Update `DecodeTeleOp.java` to use Tank Control (`gamepad1.left_stick_y`, `gamepad1.right_stick_y`).