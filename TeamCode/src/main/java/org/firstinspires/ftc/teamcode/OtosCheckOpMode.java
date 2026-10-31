package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Checks that the OTOS reports position and heading in the field convention used by
 * BiobuzzAutoDrive (x right, y away from the drivers, heading increasing turning right).
 *
 * Drive or roll the robot by hand (the drive motors float in this OpMode) and confirm:
 * - Pushing the robot forward makes Y increase.
 * - Pushing it to its right makes X increase.
 * - Turning it right (clockwise) makes the heading increase.
 * If a sign is wrong, the sensor is mounted rotated; enter the angle in
 * BiobuzzAutoDrive.OTOS_OFFSET_H.
 *
 * The Control Hub's IMU is shown beside the OTOS heading as a cross-check only. If the two
 * drift apart by a few degrees, suspect the OTOS calibration or mounting. The IMU needs the hub's
 * mounting set in LOGO_DIRECTION and USB_DIRECTION below; if the IMU is not in the configuration
 * that line is skipped.
 *
 * Controls (gamepad 1):
 * - Left stick Y / Right stick Y: tank drive
 * - Right Trigger: Gate open while held, closed when released
 * - Y: Reset the pose to (0, 0, heading 0)
 * - X: Recalibrate the OTOS gyro (keep the robot completely still)
 */
@TeleOp(name="OTOS Check", group="Test")
public class OtosCheckOpMode extends LinearOpMode {

    // Set these to match how the Control Hub is mounted on the robot
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    BiobuzzHardware robot = new BiobuzzHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        // Let the robot be rolled by hand
        robot.setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        BiobuzzAutoDrive drive = new BiobuzzAutoDrive(this, robot);
        drive.init();
        drive.setStartPose(0, 0, 0);

        IMU imu = null;
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(LOGO_DIRECTION, USB_DIRECTION)));
            imu.resetYaw();
        } catch (Exception e) {
            imu = null;
        }

        telemetry.addData("Status", "Ready. Press START.");
        telemetry.update();

        waitForStart();

        boolean prevX = false;
        boolean prevY = false;

        while (opModeIsActive()) {
            if (gamepad1.y && !prevY) {
                drive.setStartPose(0, 0, 0);
                if (imu != null) imu.resetYaw();
            }
            if (gamepad1.x && !prevX) {
                telemetry.addLine("Recalibrating - keep the robot still");
                telemetry.update();
                drive.init();
                drive.setStartPose(0, 0, 0);
                if (imu != null) imu.resetYaw();
            }
            prevX = gamepad1.x;
            prevY = gamepad1.y;

            double left = -gamepad1.left_stick_y;
            double right = -gamepad1.right_stick_y;
            robot.setDrivePower(left, left, right, right);

            // Gate is open while the right trigger is held, and closes when it is released
            robot.setGate(gamepad1.right_trigger > BiobuzzConstants.GATE_TRIGGER_THRESHOLD);

            drive.update();

            telemetry.addLine("Field pose (what the autos use)");
            drive.addPoseTelemetry();

            SparkFunOTOS.Pose2D raw = drive.otos.getPosition();
            telemetry.addLine("Raw OTOS (x right, y forward, heading counterclockwise)");
            telemetry.addData("Raw X / Y / H", "%.1f / %.1f / %.1f", raw.x, raw.y, raw.h);

            if (imu != null) {
                // The IMU turns counterclockwise-positive, so flip it to match the field heading
                double imuHeading = BiobuzzAutoDrive.normalizeDegrees(
                        -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("Hub IMU heading", "%.1f", imuHeading);
                telemetry.addData("OTOS minus IMU", "%.1f deg",
                        BiobuzzAutoDrive.normalizeDegrees(drive.getHeading() - imuHeading));
            } else {
                telemetry.addLine("Hub IMU: not available");
            }

            telemetry.addLine("Y reset pose | X recalibrate (keep still) | sticks drive");
            telemetry.update();
        }
    }
}
