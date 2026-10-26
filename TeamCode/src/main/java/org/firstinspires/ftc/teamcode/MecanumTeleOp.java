package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Mecanum drive for the BioBuzz practice bot.
 *
 * Controls (gamepad1):
 *   Left stick  - drive / strafe
 *   Right stick - rotate
 *   Right bumper (hold) - slow mode
 *   Options / Start     - reset field-centric heading
 *   Y - toggle field-centric / robot-centric
 *
 * Hardware config names (set these in the Driver Hub configuration):
 *   frontLeft, frontRight, backLeft, backRight, imu
 */
@TeleOp(name = "Mecanum Drive", group = "BioBuzz")
public class MecanumTeleOp extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    private static final double SLOW_MODE_SCALE = 0.35;
    private static final double STRAFE_CORRECTION = 1.1; // compensates for imperfect strafing

    @Override
    public void runOpMode() {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Motors on the left side face the opposite way, so reverse them.
        // If the robot drives backwards, swap FORWARD/REVERSE on all four.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Set these to match how the Control Hub is physically mounted.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        boolean fieldCentric = false;
        boolean yWasPressed = false;

        telemetry.addLine("Ready. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double y  = -gamepad1.left_stick_y;  // up on stick is negative
            double x  = gamepad1.left_stick_x * STRAFE_CORRECTION;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            if (gamepad1.y && !yWasPressed) {
                fieldCentric = !fieldCentric;
            }
            yWasPressed = gamepad1.y;

            if (fieldCentric) {
                double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
                double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
                x = rotX;
                y = rotY;
            }

            // Scale so no wheel exceeds 1.0 while keeping the ratios between wheels.
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double fl = (y + x + rx) / denominator;
            double bl = (y - x + rx) / denominator;
            double fr = (y - x - rx) / denominator;
            double br = (y + x - rx) / denominator;

            double scale = gamepad1.right_bumper ? SLOW_MODE_SCALE : 1.0;
            frontLeft.setPower(fl * scale);
            backLeft.setPower(bl * scale);
            frontRight.setPower(fr * scale);
            backRight.setPower(br * scale);

            telemetry.addData("Mode", fieldCentric ? "Field-centric" : "Robot-centric");
            telemetry.addData("Slow mode", gamepad1.right_bumper);
            telemetry.addData("FL/FR", "%.2f / %.2f", fl * scale, fr * scale);
            telemetry.addData("BL/BR", "%.2f / %.2f", bl * scale, br * scale);
            telemetry.update();
        }
    }
}
