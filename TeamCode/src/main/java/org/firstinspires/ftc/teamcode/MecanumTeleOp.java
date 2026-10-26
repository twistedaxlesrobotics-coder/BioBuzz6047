package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Mecanum drive for the BioBuzz practice bot. Drive controls match TeleopBlue.
 *
 * Driver controls (gamepad1):
 *  - Left stick: drive / strafe
 *  - Right stick X: rotate
 *  - Right trigger: slow mode
 *  - A: toggle drive direction
 *
 * Hardware config names: frontLeft, frontRight, backLeft, backRight
 */
@TeleOp(name = "Mecanum Drive", group = "BioBuzz")
public class MecanumTeleOp extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private boolean driveReversed = false;
    private boolean prevToggleA1 = false;

    @Override
    public void runOpMode() {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Match these to DecodeRobotHardware so the practice bot behaves like the main bot.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addLine("Mecanum Drive ready.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean a1 = gamepad1.a;
            if (a1 && !prevToggleA1) {
                driveReversed = !driveReversed;
                if (driveReversed) gamepad1.rumble(1.0, 1.0, 300);
                else gamepad1.rumble(0.4, 0.4, 150);
            }
            prevToggleA1 = a1;

            double drive  = -gamepad1.left_stick_y;
            double strafe =  gamepad1.left_stick_x;
            double rotate =  gamepad1.right_stick_x;

            if (driveReversed) {
                drive  = -drive;
                strafe = -strafe;
            }

            double speedModifier = 0.7 - (0.6 * gamepad1.right_trigger);

            double fl = (drive + strafe + rotate) * speedModifier;
            double fr = (drive - strafe - rotate) * speedModifier;
            double bl = (drive - strafe + rotate) * speedModifier;
            double br = (drive + strafe - rotate) * speedModifier;

            double maxMag = Math.max(1.0,
                    Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

            frontLeft.setPower(fl / maxMag);
            frontRight.setPower(fr / maxMag);
            backLeft.setPower(bl / maxMag);
            backRight.setPower(br / maxMag);

            telemetry.addData("Drive Dir", driveReversed ? "REVERSED" : "NORMAL");
            telemetry.addData("Speed modifier", "%.2f", speedModifier);
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
