package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Wheel test for the practice bot. Put the robot on blocks (wheels off the ground).
 * Hold a button to spin that wheel forward at low power with NO direction reversal:
 *   A = frontLeft, B = frontRight, X = backLeft, Y = backRight
 *
 * Every wheel should spin so the robot would drive FORWARD. Any wheel that spins
 * backward needs its direction flipped in MecanumTeleOp.
 *
 * Then look at the rollers (the angled barrels on each wheel) from above. They should
 * form an X: front-left and back-right lean one way, front-right and back-left lean the other.
 * If they form a diamond or all lean the same way, the wheels are mounted wrong and the robot
 * cannot strafe no matter what the code does.
 */
@TeleOp(name = "Mecanum Wheel Test", group = "BioBuzz")
public class MecanumWheelTest extends LinearOpMode {

    private static final double TEST_POWER = 0.3;

    @Override
    public void runOpMode() {
        DcMotor fl = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor fr = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor bl = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor br = hardwareMap.get(DcMotor.class, "backRight");

        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setDirection(DcMotorSimple.Direction.FORWARD);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addLine("Robot on blocks! Hold A/B/X/Y to spin FL/FR/BL/BR.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            fl.setPower(gamepad1.a ? TEST_POWER : 0);
            fr.setPower(gamepad1.b ? TEST_POWER : 0);
            bl.setPower(gamepad1.x ? TEST_POWER : 0);
            br.setPower(gamepad1.y ? TEST_POWER : 0);

            telemetry.addLine("A=frontLeft  B=frontRight  X=backLeft  Y=backRight");
            telemetry.addLine("Each wheel should spin so the robot drives FORWARD.");
            telemetry.update();
        }
    }
}
