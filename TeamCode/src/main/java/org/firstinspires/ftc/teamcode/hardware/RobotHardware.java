/* Copyright (c) 2025 FIRST. All rights reserved. */

package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotHardware {

    // Declare OpMode members.
    private LinearOpMode myOpMode = null; // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects
    // DRIVETRAIN
    public DcMotorEx leftFrontDrive = null;
    public DcMotorEx leftRearDrive = null;
    public DcMotorEx rightFrontDrive = null;
    public DcMotorEx rightRearDrive = null;

    // SCORING MECHANISMS
    public DcMotorEx intakeMotor = null;
    public DcMotorEx launcherMotorLeft = null;
    public DcMotorEx launcherMotorRight = null; // Optional if using dual motor launcher

    public Servo gateServo = null;
    public Servo launcherAngleServo = null;

    // SENSORS
    public IMU imu = null;
    public ColorSensor colorSensor = null;

    // Constants
    // Motor Parameters (Rev HD Hex 20:1 as example, adjust for specific motor)
    public static final double TICKS_PER_REV = 537.7;
    public static final double DRIVE_GEAR_REDUCTION = 1.0;
    public static final double WHEEL_DIAMETER_INCHES = 3.78; // 96mm
    public static final double COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_INCHES * 3.1415);

    // Servo Positions
    public static final double GATE_CLOSED = 0.0;
    public static final double GATE_OPEN = 0.5;
    public static final double LAUNCHER_ANGLE_LOW = 0.2; // Classified
    public static final double LAUNCHER_ANGLE_HIGH = 0.6; // Overflow

    // Launcher Speeds
    public static final double LAUNCHER_SPEED_CLASSIFIED = 0.85;
    public static final double LAUNCHER_SPEED_OVERFLOW = 0.95;

    // Constructor
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     */
    public void init() {
        // Define and Initialize Drivetrain Motors
        // Note: Recommended names map to what you should configure in the Driver
        // Station
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftRearDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "left_rear_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightRearDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "right_rear_drive");

        // Set Drivetrain Direction (Mecanum standard: Reverse Left side usually, but
        // check specific build)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        // Define and Initialize Mechanism Motors
        try {
            intakeMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "intake_motor");
            intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            myOpMode.telemetry.addData("Warning", "Intake Motor not found");
        }

        try {
            launcherMotorLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "launcher_left");
            launcherMotorLeft.setDirection(DcMotor.Direction.FORWARD);

            // Dual launcher motors typically run opposite
            // launcherMotorRight = myOpMode.hardwareMap.get(DcMotorEx.class,
            // "launcher_right");
            // launcherMotorRight.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            myOpMode.telemetry.addData("Warning", "Launcher Motor not found");
        }

        // Set Zero Power Behavior
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (launcherMotorLeft != null)
            launcherMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize Servos
        try {
            gateServo = myOpMode.hardwareMap.get(Servo.class, "gate_servo");
            gateServo.setPosition(GATE_CLOSED);

            launcherAngleServo = myOpMode.hardwareMap.get(Servo.class, "angle_servo");
            launcherAngleServo.setPosition(LAUNCHER_ANGLE_LOW);
        } catch (Exception e) {
            myOpMode.telemetry.addData("Warning", "Servos not found");
        }

        // Initialize IMU
        // Adjust LogoFacingDirection and UsbFacingDirection based on your Control Hub
        // mounting
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // Initialize Sensors
        try {
            colorSensor = myOpMode.hardwareMap.get(ColorSensor.class, "sensor_color");
        } catch (Exception e) {
            myOpMode.telemetry.addData("Warning", "Color Sensor not found");
        }

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Set power for all mecanum drive motors
     */
    public void setDrivePowers(double ax, double ay, double yaw) {
        double frontLeftPower = ay + ax + yaw;
        double frontRightPower = ay - ax - yaw;
        double backLeftPower = ay - ax + yaw;
        double backRightPower = ay + ax - yaw;

        // Normalize values
        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        leftFrontDrive.setPower(frontLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        leftRearDrive.setPower(backLeftPower);
        rightRearDrive.setPower(backRightPower);
    }

    /**
     * Stop all motors
     */
    public void stopAll() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
        if (intakeMotor != null)
            intakeMotor.setPower(0);
        if (launcherMotorLeft != null)
            launcherMotorLeft.setPower(0);
    }
}
