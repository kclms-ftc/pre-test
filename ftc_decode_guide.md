
# Complete FTC DECODE Robot Development Guide 2025-26

## 🎯 Game Overview: DECODE™ Presented by RTX

**Core Objectives:**
- Score purple and green ARTIFACTS into your GOAL
- Build PATTERNS on your RAMP based on the randomized MOTIF
- Return ROBOTS to BASE before time expires
- Match Format: 30s AUTO + 2min TELEOP

**Key Scoring Elements:**
- 24 Purple + 12 Green ARTIFACTS (5" polypropylene balls)
- GOAL with CLASSIFIER (SQUARE → RAMP → GATE)
- OBELISK shows 1 of 3 MOTIFS: GPP, PGP, or PPG

---

## 📋 Phase 1: Hardware Foundation (Weeks 1-3)

### 1.1 Drivetrain Selection

**RECOMMENDED: Mecanum Drive**
- **Why:** Best balance of maneuverability, field positioning, and simplicity
- **Components:**
  - 4x Mecanum wheels (goBILDA or REV)
  - 4x Motors (goBILDA 5203 series or REV HD Hex)
  - Ratio: 13.7:1 or 19.2:1 for balance of speed/power

**Alternative: Tank Drive with Omni Wheels**
- Simpler, more robust, but less maneuverable
- Good for defensive strategies

**Key Considerations for DECODE:**
- Need to navigate between LOADING ZONE and GOAL efficiently
- Ability to position precisely at GATE (within GATE ZONE)
- Fast enough to return to BASE in final 20 seconds

### 1.2 Intake/Outtake System Priority

**Critical Design Choice: Launch-Based Scoring**

**Option A: Over-the-Wall Launcher (RECOMMENDED)**
- **Mechanism:** Flywheel or catapult system
- **Requirements:**
  - Launch ARTIFACTS from LAUNCH ZONE into GOAL top opening
  - Adjustable angle/power for CLASSIFIED vs OVERFLOW scoring
  - Must handle both purple and green balls consistently
  
**Key Components:**
- 2x Motors for dual-flywheel (Andymark NeveRest or goBILDA 5203)
- Angled hood to direct shots
- Proximity sensor to detect ball presence

**Option B: Direct Deposit Mechanism**
- Arm that reaches over GOAL wall to drop ARTIFACTS
- More controlled but slower cycle time
- Requires vertical lift mechanism

### 1.3 Intake System

**Roller Intake (RECOMMENDED)**
- 2x continuous compliant wheels
- 1x Motor for intake
- Surgical tubing or compliant wheels for grip
- Wide mouth (7-8") to capture balls from floor

**Placement:** Front of robot, low to ground (within 2" of tiles)

### 1.4 GATE Interaction Mechanism

**CRITICAL GAME ELEMENT**
- Must push GATE lever to release CLASSIFIED ARTIFACTS
- GATE contact point: 3.75" - 5.5" above tiles when closed
- Requires ~2" horizontal displacement to open

**Design Approach:**
- **Simple pusher:** Vertical panel 6" tall, servo or pneumatic actuated
- **Active holder:** Mechanism that can hold GATE open to clear all ARTIFACTS
- Mount on front or side of robot for consistent access

### 1.5 Motor & Servo Allocation

**Motor Budget (8 total):**
- 4x Drivetrain motors
- 2x Launcher/Flywheel motors
- 1x Intake motor
- 1x Optional: Linear slide or arm extension

**Servo Budget (10 total):**
- 2x GATE pusher servos (redundancy)
- 2x Intake positioning/gates
- 2x Launcher angle adjustment
- 4x Reserved for additional mechanisms

### 1.6 Electronics Setup

**Control System:**
- REV Control Hub (primary) + REV Expansion Hub (optional)
- 12V NiMH Battery (REV or goBILDA)
- Main power switch with 20A fuse
- All servos must comply with 8W@6V limit

**Sensors (CRITICAL for AUTO):**
- 2x Color sensors (detecting ARTIFACT colors)
- 1x IMU (heading control, comes with Control Hub)
- 1x Distance sensor (GOAL/wall detection)
- Optional: Webcam for AprilTag detection (OBELISK MOTIF reading)

### 1.7 Size Constraints

**Starting Configuration:** 18" cube (45.7cm)
**During Match:**
- Horizontal: Fixed 18" x 18" footprint (NO software limits allowed)
- Vertical: 18" base limit, can extend to 38" in final 20 seconds

---

## 🏗️ Phase 2: Mechanical Design Details (Weeks 3-5)

### 2.1 CAD and Prototyping

**Recommended Tools:**
- OnShape (free, cloud-based, FTC library available)
- Fusion 360 (free for students)

**Design Priorities:**
1. Robust drivetrain mounting
2. Intake-to-launcher ball pathway
3. GATE interaction clearance
4. Center of gravity below 9" for stability

### 2.2 Launcher Optimization

**Flywheel Design:**
- Dual 4" wheels with high-friction surface
- Compression: 1/4" to 1/2" ball compression
- RPM: 2500-3500 for GOAL distance (~6-8 feet)
- Hood angle: 35-45 degrees

**Adjustability Features:**
- Servo-controlled hood for CLASSIFIED vs OVERFLOW targeting
- Variable speed control for distance compensation

### 2.3 Ball Management

**Internal Storage:**
- Hold 3 balls max per G408 rules
- Indexer to feed launcher one ball at a time
- Sensor-based detection to prevent jams

### 2.4 ROBOT Sign Compliance

**Requirements (R401-R403):**
- 2 signs, ≥90° apart, 6.5" x 2.5" minimum
- Solid red or blue background
- White team number 2.25" ± 0.5" tall
- Robust material (acrylic, polycarbonate)

---

## 💻 Phase 3: Software Development (Weeks 4-8)

### 3.1 Development Environment Setup

**SDK Version:**
- FTC SDK 11.0 (minimum recommended)
- Control Hub OS 1.1.2
- Hub Firmware 1.8.2

**Programming Options:**
1. **Java (Recommended for complex logic)**
2. **Blocks (Good for beginners)**

**Version Control:**
- GitHub repository for team code
- Regular commits with descriptive messages

### 3.2 Autonomous Programming Order

#### **Step 1: Basic Movement (Week 4)**

```java
// Encoder-based driving
public void driveForward(double inches, double power) {
    int targetTicks = (int)(inches * COUNTS_PER_INCH);
    resetEncoders();
    setTargetPosition(targetTicks);
    setMode(RUN_TO_POSITION);
    setPower(power);
    while (motorsAreBusy() && opModeIsActive()) {
        telemetry.addData("Position", getPosition());
        telemetry.update();
    }
    stopMotors();
}
```

**Test Goals:**
- Drive forward 24", 48", 72"
- Turn 90°, 180° accurately
- Strafe left/right (if mecanum)

#### **Step 2: IMU Integration (Week 4-5)**

```java
// Heading-based turning
public void turnToHeading(double targetHeading, double power) {
    BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
    double currentHeading = getHeading(imu);
    double error = targetHeading - currentHeading;
    
    while (Math.abs(error) > 2 && opModeIsActive()) {
        double turnPower = Range.clip(error * 0.02, -power, power);
        leftDrive.setPower(-turnPower);
        rightDrive.setPower(turnPower);
        
        currentHeading = getHeading(imu);
        error = targetHeading - currentHeading;
    }
    stopMotors();
}
```

#### **Step 3: AprilTag Vision (Week 5-6)**

**MOTIF Detection from OBELISK:**
- Use webcam to detect AprilTag ID (21, 22, or 23)
- Map ID to MOTIF: 21=GPP, 22=PGP, 23=PPG

```java
// AprilTag detection setup
AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
    .setDrawAxes(true)
    .setDrawCubeProjection(true)
    .setDrawTagOutline(true)
    .build();

VisionPortal visionPortal = new VisionPortal.Builder()
    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
    .addProcessor(tagProcessor)
    .build();
```

**AUTO Sequence:**
1. Read OBELISK AprilTag at start
2. Store MOTIF pattern (GPP/PGP/PPG)
3. Navigate to scoring positions based on MOTIF

#### **Step 4: Sensor-Based Scoring (Week 6-7)**

```java
// Color sensor for ball detection
ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "color");

public String detectBallColor() {
    int red = colorSensor.red();
    int green = colorSensor.green();
    int blue = colorSensor.blue();
    
    if (red > green && red > blue) return "PURPLE";
    if (green > red && green > blue) return "GREEN";
    return "UNKNOWN";
}
```

#### **Step 5: Complete AUTO Routine (Week 7-8)**

**HIGH-PRIORITY AUTO Path:**
1. **Pre-load score (0-5s):** Launch 2-3 pre-loaded balls
2. **LEAVE bonus (5-10s):** Move off LAUNCH LINE
3. **Additional scoring (10-25s):** Collect + score 2-3 more balls
4. **PATTERN attempt (25-30s):** Score correct color in SQUARE based on MOTIF

**Expected AUTO Points:** 35-50 points
- LEAVE: 3pts
- 5 ARTIFACTS CLASSIFIED: 15pts
- 3 PATTERN matches: 6pts
- Movement RP: 21pts total needed (combine LEAVE + TELEOP BASE)

### 3.3 TeleOp Programming Order

#### **Step 1: Drivetrain Control (Week 5)**

```java
// Mecanum drive with field-centric option
double drive = -gamepad1.left_stick_y;
double strafe = gamepad1.left_stick_x;
double turn = gamepad1.right_stick_x;

// Optional: IMU-based field-centric driving
double heading = imu.getAngularOrientation().firstAngle;
double rotated_drive = drive * Math.cos(heading) - strafe * Math.sin(heading);
double rotated_strafe = drive * Math.sin(heading) + strafe * Math.cos(heading);

frontLeft.setPower(rotated_drive + rotated_strafe + turn);
frontRight.setPower(rotated_drive - rotated_strafe - turn);
backLeft.setPower(rotated_drive - rotated_strafe + turn);
backRight.setPower(rotated_drive + rotated_strafe - turn);
```

#### **Step 2: Launcher Control (Week 6)**

```java
// Flywheel control with ramping
private double targetFlywheelSpeed = 0;
private double currentFlywheelSpeed = 0;

public void runFlywheelControl() {
    if (gamepad2.a) {
        targetFlywheelSpeed = 0.85; // Classified scoring
    } else if (gamepad2.b) {
        targetFlywheelSpeed = 0.95; // Overflow scoring
    } else if (gamepad2.x) {
        targetFlywheelSpeed = 0;
    }
    
    // Smooth ramping
    currentFlywheelSpeed += (targetFlywheelSpeed - currentFlywheelSpeed) * 0.1;
    flywheelMotor.setPower(currentFlywheelSpeed);
    
    // Fire when up to speed
    if (gamepad2.right_bumper && Math.abs(currentFlywheelSpeed - targetFlywheelSpeed) < 0.05) {
        feedBall();
    }
}
```

#### **Step 3: Intake Control (Week 6)**

```java
// Simple intake toggle
if (gamepad2.left_bumper) {
    intakeMotor.setPower(1.0);  // Intake
} else if (gamepad2.left_trigger > 0.1) {
    intakeMotor.setPower(-1.0); // Eject
} else {
    intakeMotor.setPower(0);
}
```

#### **Step 4: GATE Control (Week 7)**

```java
// GATE servo positions
private static final double GATE_CLOSED = 0.0;
private static final double GATE_OPEN = 0.5;

if (gamepad2.dpad_right) {
    gateServo.setPosition(GATE_OPEN);
    sleep(500); // Hold open to clear RAMP
    gateServo.setPosition(GATE_CLOSED);
}
```

### 3.4 Advanced Features (Weeks 8-12)

#### **Feature 1: Vision-Assisted Alignment**
- Use AprilTag on GOAL (ID 20 for blue, ID 24 for red)
- Auto-align robot to optimal scoring position

#### **Feature 2: Automated Scoring Sequence**
```java
public void autoScore() {
    // 1. Spin up flywheel
    setFlywheelSpeed(0.85);
    waitForFlywheelSpeed();
    
    // 2. Feed 3 balls with timing
    for (int i = 0; i < 3; i++) {
        feedBall();
        sleep(300); // Time between shots
    }
    
    // 3. Spin down
    setFlywheelSpeed(0);
}
```

#### **Feature 3: Endgame Timer**
```java
// Visual/audio warning for BASE return
double timeRemaining = 120 - getRuntime();
if (timeRemaining < 20 && !endgameWarning) {
    gamepad1.rumble(1000);
    gamepad2.rumble(1000);
    endgameWarning = true;
}

// Auto-drive to BASE on button press
if (gamepad1.y && timeRemaining < 20) {
    driveToBase();
}
```

#### **Feature 4: Telemetry Dashboard**
```java
telemetry.addData("Flywheel Speed", currentFlywheelSpeed);
telemetry.addData("Balls in Robot", ballCount);
telemetry.addData("Time Remaining", timeRemaining);
telemetry.addData("AUTO Motif", currentMotif);
telemetry.update();
```

---

## 🎨 Phase 4: Advanced Strategy & Optimization (Weeks 9-15)

### 4.1 Scoring Strategy Matrix

**CLASSIFIED vs OVERFLOW Decision:**
- **CLASSIFIED (3pts each):** Controlled, sequential scoring
- **OVERFLOW (1pt each):** Faster cycle, less precision

**When to use each:**
- First 9 shots: Always aim for CLASSIFIED
- After RAMP full: Continue OVERFLOW or open GATE
- AUTO: Always CLASSIFIED for PATTERN points

### 4.2 PATTERN Strategy

**MOTIF Memorization:**
- GPP: Green-Purple-Purple (most common at 2/3 probability)
- PGP: Purple-Green-Purple
- PPG: Purple-Purple-Green

**Scoring Approach:**
1. Identify MOTIF in AUTO (or first 10s of TELEOP)
2. Pre-sort balls: Have 3 correct sequence ready
3. Score pattern index 1-3 first, then continue cycling
4. Don't attempt pattern if >20s of match remaining (too risky)

### 4.3 Cycle Time Optimization

**Target Cycle Times:**
- Pick up ball from LOADING ZONE: 2-3s
- Navigate to LAUNCH ZONE: 3-4s
- Launch ball: 1-2s
- **Total cycle: 6-9 seconds per ball**

**Goal:** 12-15 cycles in 2min TELEOP = 36-45 balls scored

### 4.4 Defensive Play (Optional)

**Legal Defensive Actions:**
- Contest LOADING ZONE access (stay in your own)
- Overflow balls into opponent's SECRET TUNNEL
- Strategic GATE interference (must avoid G417 penalty)

**Illegal Actions (AVOID):**
- Contacting opponent in their GATE ZONE (G424)
- Contacting opponent in their SECRET TUNNEL (G425)
- Damaging opponent ROBOT (G420)

### 4.5 Endgame Tactics

**BASE Scoring (Final 20s):**
- **Partial BASE (5pts):** Some support from BASE ZONE
- **Full BASE (10pts):** Only supported by BASE ZONE tiles
- **Both ROBOTS Full (20pts total + 10 bonus):** Coordinate with partner

**Expansion Rules:**
- Can expand to 38" tall in final 20s only (G415)
- Use this for reaching high DEPOT scoring or intimidation

---

## 📚 Phase 5: Inspiration & Resources

### 5.1 Top Teams to Study

**YouTube Channels:**
- **FTC Team 731 "Wannabee Strange"** - Exceptional programming tutorials
- **FTC Team 7203 "KNO3"** - Hardware design breakdowns
- **FTC Team 8680 "Kraken Pinion"** - CAD design series
- **gm0 (Game Manual 0)** - Community-driven FTC resource

### 5.2 Essential Software Libraries

**FTCLib** (Recommended)
- Command-based programming framework
- Advanced drive classes (MecanumDrive, etc.)
- Built-in sensor fusion
- Install: Add to build.gradle

**RoadRunner** (For Advanced AUTO)
- Spline-based path following
- Accurate localization using odometry
- Smooth, fast autonomous movements

**EasyOpenCV** (For Vision)
- Simplified OpenCV integration
- AprilTag detection pipelines
- Custom vision processing

### 5.3 Community Resources

**Forums & Websites:**
- FTC Discord Server (largest FTC community)
- Reddit: r/FTC
- Chief Delphi Forums (cross-FRC collaboration)
- gm0.org (game manual zero - comprehensive guides)

**Vendor Resources:**
- REV Robotics Learning Site
- goBILDA Build System Documentation
- AndyMark Tech Support

### 5.4 Testing & Competition Prep

**Week 13-15: Testing Protocol**
1. **Durability Testing:** Run robot for 30+ matches
2. **Driver Practice:** 20+ hours of controlled practice
3. **AUTO Reliability:** 95%+ success rate required
4. **Match Simulation:** Full 2:30 timed runs with scoring

**Pre-Competition Checklist:**
- [ ] Spare parts kit (gears, wheels, electronics)
- [ ] Extra charged batteries (6+ minimum)
- [ ] Printed robot sign (red & blue versions)
- [ ] Team portfolio for judging
- [ ] Driver playbook with strategy charts

---

## 🏆 Winning Robot Profile (Target Specs)

**Performance Goals:**

| Metric | Target | Elite |
|--------|--------|-------|
| AUTO Score | 40+ pts | 55+ pts |
| TELEOP Score | 80+ pts | 120+ pts |
| Cycle Time | <9s | <7s |
| PATTERN Success | 75% | 95% |
| BASE Return | 100% | 100% |

**Competitive Features Priority:**
1. ✅ Consistent launcher (95%+ accuracy)
2. ✅ Fast intake system (<3s pickup)
3. ✅ Reliable GATE mechanism
4. ✅ Accurate AUTO with MOTIF detection
5. ✅ Robust drivetrain (survives contact)
6. ✅ Quick BASE return capability
7. ⚡ Vision-assisted scoring (optional)
8. ⚡ Automated scoring macros (optional)

---

## 📋 Development Timeline Summary

| Weeks | Phase | Deliverables |
|-------|-------|--------------|
| 1-3 | Hardware Design | CAD complete, parts ordered |
| 3-5 | Build & Assembly | Functional robot chassis |
| 4-8 | Software Core | AUTO + TeleOp basic functions |
| 6-8 | Integration | Sensors + vision working |
| 8-12 | Advanced Features | Macros, optimization |
| 9-15 | Testing & Iteration | Competition-ready robot |

**Key Milestones:**
- Week 6: First successful AUTO run
- Week 8: Score 30+ points in practice match
- Week 10: Reliable PATTERN scoring
- Week 12: Full match simulation >100 pts
- Week 15: Competition ready

---

## 🎯 Final Tips for Success

1. **Start Simple:** Get basic driving + launcher working first
2. **Iterate Rapidly:** Test → Identify Issues → Fix → Repeat
3. **Document Everything:** Code comments, build photos, design decisions
4. **Practice Driving:** 50+ hours of controlled practice for drivers
5. **Study Rules:** Know penalty rules (G408, G414, G415, G420) by heart
6. **Build for Durability:** Robots take hits - design accordingly
7. **Have Fun:** Remember FIRST Core Values and Gracious Professionalism

**Most Important:** The best robot is the one that works reliably. Focus on executing core tasks well rather than attempting complex features that may fail under pressure.

Good luck with DECODE 2025-26! 🚀