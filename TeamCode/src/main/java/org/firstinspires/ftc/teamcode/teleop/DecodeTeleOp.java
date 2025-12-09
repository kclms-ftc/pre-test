package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp(name = "DECODE TeleOp", group = "Decoded")
public class DecodeTeleOp extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    // Timer for endgame warning
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize the robot hardware.
        robot.init();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // --- DRIVER 1: CHASSIS CONTROL ---

            // Mecanum Drive Controls (Field Centric)
            // Left Stick: Drive (Translation)
            // Right Stick: Turn (Rotation)

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Retrieve the robot's heading from the IMU
            double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Apply powers using the RobotHardware helper
            robot.setDrivePowers(rotX, rotY, rx);

            // Quick Reset of Heading for Field Centric (Driver 1 Option button)
            if (gamepad1.options) {
                robot.imu.resetYaw();
            }

            // --- DRIVER 2: MECHANISMS ---

            // Intake Control
            // Right Trigger: Intake
            // Left Trigger: Eject
            if (robot.intakeMotor != null) {
                if (gamepad2.right_trigger > 0.1) {
                    robot.intakeMotor.setPower(1.0);
                } else if (gamepad2.left_trigger > 0.1) {
                    robot.intakeMotor.setPower(-1.0);
                } else {
                    robot.intakeMotor.setPower(0);
                }
            }

            // Launcher Control
            // Button A: Classified Speed (Targeted)
            // Button B: Overflow Speed (Hard shot)
            // Button X: Stop Launcher
            if (robot.launcherMotorLeft != null) {
                if (gamepad2.a) {
                    robot.launcherMotorLeft.setPower(RobotHardware.LAUNCHER_SPEED_CLASSIFIED);
                    if (robot.launcherAngleServo != null)
                        robot.launcherAngleServo.setPosition(RobotHardware.LAUNCHER_ANGLE_LOW);
                } else if (gamepad2.b) {
                    robot.launcherMotorLeft.setPower(RobotHardware.LAUNCHER_SPEED_OVERFLOW);
                    if (robot.launcherAngleServo != null)
                        robot.launcherAngleServo.setPosition(RobotHardware.LAUNCHER_ANGLE_HIGH);
                } else if (gamepad2.x) {
                    robot.launcherMotorLeft.setPower(0);
                }
            }

            // Gate Control
            // D-Pad Up: Open Gate
            // D-Pad Down: Close Gate
            if (robot.gateServo != null) {
                if (gamepad2.dpad_up) {
                    robot.gateServo.setPosition(RobotHardware.GATE_OPEN);
                } else if (gamepad2.dpad_down) {
                    robot.gateServo.setPosition(RobotHardware.GATE_CLOSED);
                }
            }

            // --- TELEMETRY ---
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Heading", "%4.2f rad", botHeading);

            // Show Sensor Data
            if (robot.colorSensor != null) {
                telemetry.addData("Red", robot.colorSensor.red());
                telemetry.addData("Green", robot.colorSensor.green());
                telemetry.addData("Blue", robot.colorSensor.blue());
            }

            // Endgame Warning
            if (runtime.seconds() > 90) { // Last 30 seconds
                telemetry.addData("WARNING", "ENDGAME STARTED - Return to Base!");
            }

            telemetry.update();
        }
    }
}
