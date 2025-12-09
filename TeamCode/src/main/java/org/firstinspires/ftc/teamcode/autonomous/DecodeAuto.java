package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "DECODE Auto (Blue/Red)", group = "Decoded")
public class DecodeAuto extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    private ElapsedTime runtime = new ElapsedTime();

    // Vision
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Game Constants
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final int TAG_ID_GPP = 21;
    static final int TAG_ID_PGP = 22;
    static final int TAG_ID_PPG = 23;

    @Override
    public void runOpMode() {
        // Init Hardware
        robot.init();

        // Init Vision
        initAprilTag();

        // Wait for Start
        // Loop during Init to detect Tag
        int detectedId = -1;

        telemetry.addData("Status", "Ready to Run");
        telemetry.addData("Vision", "Looking for tags 21, 22, 23...");
        telemetry.update();

        while (opModeInInit()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if (detection.id == TAG_ID_GPP || detection.id == TAG_ID_PGP || detection.id == TAG_ID_PPG) {
                        detectedId = detection.id;
                        telemetry.addData("DETECTED MOTIF ID", detectedId);
                        break;
                    }
                }
            }
            telemetry.update();
        }

        // --- GAME START ---
        waitForStart();

        // Stop Streaming to save CPU
        // visionPortal.stopStreaming();

        telemetry.addData("Path", "Started");
        telemetry.update();

        // 1. Drive Forward off Wall (Leave Zone)
        // Assume starting facing OBELISK
        driveStraight(DRIVE_SPEED, 24.0, 5.0); // Drive 24 inches forward

        // 2. Logic based on Motif
        if (detectedId == TAG_ID_GPP) {
            telemetry.addData("Strategy", "Executing GPP Pattern");
            // Add specific logic for GPP position
        } else if (detectedId == TAG_ID_PGP) {
            telemetry.addData("Strategy", "Executing PGP Pattern");
        } else {
            telemetry.addData("Strategy", "Default / Unknown Tag");
        }
        telemetry.update();

        // 3. Score Pre-Loaded Pixel (Shoot)
        if (robot.launcherMotorLeft != null) {
            // Spin up
            robot.launcherMotorLeft.setPower(RobotHardware.LAUNCHER_SPEED_CLASSIFIED);
            sleep(1500); // Wait for spin up

            // Feed (Pulse Intake or Feeder Servo if existed, usually logic is here)
            // For now, just spin and wait

            // Spin down
            robot.launcherMotorLeft.setPower(0);
        }

        // 4. Park
        // Strafe or turn to park
        // For Mecanum, we can strafe. For simpler, just turn and park.

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        try {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } catch (Exception e) {
            telemetry.addData("Vision Error", "Camera not found");
        }
    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion. Should allow for _/-
     *                 variance for adjusting heading
     * @param distance Distance (in inches) to move from current position. Negative
     *                 distance means move backwards.
     * @param timeoutS maximum execution time for this move.
     */
    public void driveStraight(double speed, double distance, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int moveCounts;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * RobotHardware.COUNTS_PER_INCH);
            newLeftTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightFrontDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFrontDrive.setTargetPosition(newLeftTarget);
            robot.rightFrontDrive.setTargetPosition(newRightTarget);
            robot.leftRearDrive.setTargetPosition(newLeftTarget);
            robot.rightRearDrive.setTargetPosition(newRightTarget);

            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            robot.setDrivePowers(speed, speed, 0); // No turn

            // keep looping while we are still active, and BOTH motors are running.
            runtime.reset();
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.stopAll();

            // Turn off RUN_TO_POSITION
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
