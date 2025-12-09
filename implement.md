# FTC DECODE: Robot Software Implementation Guide

This guide explains **exactly** how to set up, configure, and run the software provided in the `TeamCode` folder. Follow these steps to ensure your robot works successfully on the field.

---

## 1. Understanding the Programs
We have created two main programs (OpModes) for your robot. You will need to select the correct one depending on the match phase.

### **A. Autonomous Phase (Top Priority)**
*   **File:** `DecodeAuto.java`
*   **Name on Phone:** `DECODE Auto (Blue/Red)`
*   **When to run:** The first 30 seconds of the match.
*   **What it does:**
    1.  Uses the Webcam to read the **AprilTag** on the OBELISK.
    2.  Decides which **MOTIF** (Pattern) to play (GPP, PGP, or PPG).
    3.  Drives forward to the Scoring Zone.
    4.  Automatically launches the pre-loaded Purple Pixel.
*   **No Driver Input:** You press "Init", wait for the camera to see the tag, then press "Start". Hands off!

### **B. TeleOp Phase (Driver Control)**
*   **File:** `DecodeTeleOp.java`
*   **Name on Phone:** `DECODE TeleOp`
*   **When to run:** The remaining 2 minutes of the match (after Auto ends).
*   **What it does:** Gives full control to the drivers.
    *   **Driver 1 (Gamepad 1):** Moves the robot. Uses **Field-Centric Driving** (pushing stick forward always moves robot away from you, regardless of rotation).
    *   **Driver 2 (Gamepad 2):** Controls the Launcher, Intake, and Gate.

---

## 2. "How to Make It Work": Configuration Guide
**CRITICAL STEP:** The code will *crash* if the hardware map on the Robot Controller does not match the names in the code *exactly*.

1.  Turn on your Robot Controller (Control Hub) and Driver Station (Phone/Hub).
2.  On the Driver Station, go to **Settings** -> **Configure Robot**.
3.  **New Configuration** -> **Scan**.
4.  Select the **Control Hub Portal** -> **Control Hub**.
5.  **Configure Motors (Ports 0-3):**
    *   Port 0: type `left_front_drive`
    *   Port 1: type `left_rear_drive`
    *   Port 2: type `right_front_drive`
    *   Port 3: type `right_rear_drive`
    *   *Note:* If you are using an Expansion Hub, these might be split. Ensure you know which motor is physically plugged where!
6.  **Configure Expansion Hub (if used):**
    *   configure Motors for:
        *   `intake_motor`
        *   `launcher_left`
7.  **Configure Servos (Control Hub or Expansion Hub):**
    *   Port 0: type `gate_servo`
    *   Port 1: type `angle_servo` (if used)
8.  **Configure Sensors (I2C Bus):**
    *   Port 0: `imu` (Built-in usually, but check config)
    *   Port 1: `sensor_color`
9.  **Configure Camera:**
    *   Add "Webcam" and name it `Webcam 1`.
10. **Save Configuration:** Name it "DECODE_BOT". Activate it.

---

## 3. Deployment Steps
How to get the code from your computer to the robot.

### Option A: Android Studio (Recommended)
1.  Connect computer to Control Hub via USB or Wi-Fi.
2.  Click the Green **Run** (Play) button in Android Studio.
3.  Wait for the "Launch Succeeded" message.

### Option B: OnBot Java
1.  Connect to the robot's Wi-Fi network.
2.  Go to `192.168.43.1:8080` in your browser.
3.  Upload the `.java` files from the `TeamCode` folder.
4.  Click **Build Everything**.

---

## 4. Match Flow: How to Run
### Before the Match
1.  Place Robot on Field.
2.  **Turn ON** the robot.
3.  **Gamepad 1** (Driver): Press `Start` + `A`.
4.  **Gamepad 2** (Operator): Press `Start` + `B`.
5.  On Driver Station, select **"DECODE Auto"**.
6.  Press **INIT**.
    *   *Check Telemetry:* Does it say "DETECTED MOTIF ID"? If not, adjust robot position so camera sees the tag.

### The Match
1.  **3-2-1 GO:** Press **Play** (Triangle).
    *   Robot runs Auto.
    *   When timer hits 30s, robot stops.
2.  **Transition:**
    *   Select **"DECODE TeleOp"** immediately.
    *   Press **INIT**.
    *   Wait for "TeleOp Start" whistle.
    *   Press **Play**.
3.  **Driving:**
    *   If the robot drives "weirdly" (directions wrong), Press **Options** on Gamepad 1 to reset the Field-Centric heading.

---

## 5. Troubleshooting
*   **"Error: Hardware Missing..."**: You spelled a name wrong in the Configuration (Step 2). It must depend on capitalization! `left_front_drive` is not `Left_Front_Drive`.
*   **Robot spins when pushing forward**: Your motor directions are reversed. In `RobotHardware.java`, change `REVERSE` to `FORWARD` for the affected side.
*   **Launcher doesn't spin**: Check the battery voltage. If < 12V, motors might struggle.
*   **Code Crashing**: Check the "Log" on the Driver Station for the specific line number error.
