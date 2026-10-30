package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Helps tune the OTOS scalars and the BiobuzzAutoDrive drive and turn gains.
 *
 * Three modes, cycled with B (gamepad 1). In the drive and turn modes the robot's current facing is
 * treated as heading 0 each time a test runs, so just point the robot down a clear stretch of floor
 * (the longest test is 96 inches) and press A. Press the gamepad Stop button at any time.
 *
 * SCALARS (drive motors float - push the robot by hand):
 * - Y: Mark the start (zeroes the pose).
 * - Push the robot a measured distance along a tape measure and/or rotate it by a measured angle.
 * - D-pad Up / Down: actual distance +/- 1 in. D-pad Right / Left: actual angle +/- 1 deg.
 * - The OpMode shows the scalars to try, which are the current constants times actual over reported.
 *   Repeat a few times, average, and enter the result in BiobuzzAutoDrive.OTOS_LINEAR_SCALAR and
 *   OTOS_ANGULAR_SCALAR.
 *
 * DRIVE: A runs driveForward(distance); D-pad Up / Down changes the distance by 6 in.
 * TURN:  A runs turnToHeading(angle);    D-pad Up / Down changes the angle by 15 deg.
 * In both: Left / Right Bumper selects a gain and D-pad Left / Right lowers or raises it. Gains
 * changed here last until the app restarts, so copy the values you like into BiobuzzAutoDrive.
 *
 * The sensor is the only thing measuring, so check the result against a tape measure or a marked
 * angle on the floor. A big difference means the scalar or the offset still needs work.
 *
 * Other controls: X recalibrates the OTOS gyro (keep the robot completely still).
 */
@TeleOp(name = "OTOS Tuning", group = "Test")
public class OtosTuningOpMode extends LinearOpMode {

    private enum Mode { SCALARS, DRIVE, TURN }

    private static final String[] DRIVE_GAINS = {
            "DRIVE_KP", "DRIVE_MIN_POWER", "DRIVE_MAX_POWER", "DRIVE_RAMP_SEC", "HOLD_KP"
    };
    private static final String[] TURN_GAINS = {
            "TURN_KP", "TURN_MIN_POWER", "TURN_MAX_POWER", "TURN_TOLERANCE_DEG"
    };

    private static final double DISTANCE_STEP = 6.0;
    private static final double DISTANCE_MIN = 6.0;
    private static final double DISTANCE_MAX = 96.0;
    private static final double ANGLE_STEP = 15.0;

    BiobuzzHardware robot = new BiobuzzHardware();

    private Mode mode = Mode.SCALARS;
    private int gainIndex = 0;
    private double testDistance = 36.0;
    private double testAngle = 90.0;
    private double actualDistance = 36.0;
    private double actualAngle = 90.0;
    private String lastResult = "No test run yet";

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        BiobuzzAutoDrive drive = new BiobuzzAutoDrive(this, robot);
        drive.init();
        drive.setStartPose(0, 0, 0);
        applyMode(drive);

        telemetry.addData("Status", "Ready. Press START.");
        telemetry.update();

        waitForStart();

        boolean prevA = false, prevB = false, prevX = false, prevY = false;
        boolean prevUp = false, prevDown = false, prevLeft = false, prevRight = false;
        boolean prevLb = false, prevRb = false;

        while (opModeIsActive()) {
            boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x, y = gamepad1.y;
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
            boolean lb = gamepad1.left_bumper, rb = gamepad1.right_bumper;

            if (b && !prevB) {
                mode = Mode.values()[(mode.ordinal() + 1) % Mode.values().length];
                gainIndex = 0;
                applyMode(drive);
            }

            if (x && !prevX) {
                telemetry.addLine("Recalibrating - keep the robot still");
                telemetry.update();
                drive.init();
                drive.setStartPose(0, 0, 0);
            }

            switch (mode) {
                case SCALARS:
                    if (y && !prevY) drive.setStartPose(0, 0, 0);
                    if (up && !prevUp) actualDistance += 1.0;
                    if (down && !prevDown) actualDistance = Math.max(1.0, actualDistance - 1.0);
                    if (right && !prevRight) actualAngle += 1.0;
                    if (left && !prevLeft) actualAngle = Math.max(1.0, actualAngle - 1.0);
                    break;

                case DRIVE:
                    if (up && !prevUp) testDistance = Math.min(DISTANCE_MAX, testDistance + DISTANCE_STEP);
                    if (down && !prevDown) testDistance = Math.max(DISTANCE_MIN, testDistance - DISTANCE_STEP);
                    adjustGains(DRIVE_GAINS, lb && !prevLb, rb && !prevRb,
                            left && !prevLeft, right && !prevRight);
                    if (a && !prevA) runDriveTest(drive);
                    break;

                case TURN:
                    if (up && !prevUp) testAngle = BiobuzzAutoDrive.normalizeDegrees(testAngle + ANGLE_STEP);
                    if (down && !prevDown) testAngle = BiobuzzAutoDrive.normalizeDegrees(testAngle - ANGLE_STEP);
                    adjustGains(TURN_GAINS, lb && !prevLb, rb && !prevRb,
                            left && !prevLeft, right && !prevRight);
                    if (a && !prevA) runTurnTest(drive);
                    break;
            }

            prevA = a; prevB = b; prevX = x; prevY = y;
            prevUp = up; prevDown = down; prevLeft = left; prevRight = right;
            prevLb = lb; prevRb = rb;

            showTelemetry(drive);
        }
        drive.stop();
    }

    private void applyMode(BiobuzzAutoDrive drive) {
        drive.stop();
        // Float in the hand-push mode so the robot rolls freely; brake for the driven tests
        robot.setDriveZeroPowerBehavior(mode == Mode.SCALARS
                ? DcMotor.ZeroPowerBehavior.FLOAT
                : DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setStartPose(0, 0, 0);
    }

    private void runDriveTest(BiobuzzAutoDrive drive) {
        drive.setStartPose(0, 0, 0);
        long startNs = System.nanoTime();
        boolean reached = drive.driveForward(testDistance, 10.0);
        double seconds = (System.nanoTime() - startNs) / 1e9;
        sleep(500);   // let the robot settle before reading the final pose
        drive.update();
        double traveled = Math.hypot(drive.getX(), drive.getY());
        lastResult = String.format("Drive %.0f in: %s in %.2f s. OTOS says %.1f in, heading off %.1f deg. "
                        + "Measure the real distance with a tape.",
                testDistance, reached ? "reached" : "TIMED OUT", seconds, traveled,
                BiobuzzAutoDrive.normalizeDegrees(drive.getHeading()));
    }

    private void runTurnTest(BiobuzzAutoDrive drive) {
        drive.setStartPose(0, 0, 0);
        long startNs = System.nanoTime();
        boolean reached = drive.turnToHeading(testAngle, 5.0);
        double seconds = (System.nanoTime() - startNs) / 1e9;
        sleep(500);   // let the robot settle before reading the final pose
        drive.update();
        double error = BiobuzzAutoDrive.normalizeDegrees(testAngle - drive.getHeading());
        lastResult = String.format("Turn to %.0f deg: %s in %.2f s. OTOS says %.1f deg (error %.1f). "
                        + "Check the real angle against a floor mark.",
                testAngle, reached ? "reached" : "TIMED OUT", seconds, drive.getHeading(), error);
    }

    private void adjustGains(String[] gains, boolean prev, boolean next, boolean lower, boolean raise) {
        if (next) gainIndex = (gainIndex + 1) % gains.length;
        if (prev) gainIndex = (gainIndex + gains.length - 1) % gains.length;
        if (lower) changeGain(gains[gainIndex], -1);
        if (raise) changeGain(gains[gainIndex], +1);
    }

    private void changeGain(String name, int direction) {
        switch (name) {
            case "DRIVE_KP":
                BiobuzzAutoDrive.DRIVE_KP = Math.max(0.0, BiobuzzAutoDrive.DRIVE_KP + direction * 0.005);
                break;
            case "DRIVE_MIN_POWER":
                BiobuzzAutoDrive.DRIVE_MIN_POWER = clamp(BiobuzzAutoDrive.DRIVE_MIN_POWER + direction * 0.01, 0.0, 1.0);
                break;
            case "DRIVE_MAX_POWER":
                BiobuzzAutoDrive.DRIVE_MAX_POWER = clamp(BiobuzzAutoDrive.DRIVE_MAX_POWER + direction * 0.05, 0.1, 1.0);
                break;
            case "DRIVE_RAMP_SEC":
                BiobuzzAutoDrive.DRIVE_RAMP_SEC = clamp(BiobuzzAutoDrive.DRIVE_RAMP_SEC + direction * 0.05, 0.05, 2.0);
                break;
            case "HOLD_KP":
                BiobuzzAutoDrive.HOLD_KP = Math.max(0.0, BiobuzzAutoDrive.HOLD_KP + direction * 0.002);
                break;
            case "TURN_KP":
                BiobuzzAutoDrive.TURN_KP = Math.max(0.0, BiobuzzAutoDrive.TURN_KP + direction * 0.001);
                break;
            case "TURN_MIN_POWER":
                BiobuzzAutoDrive.TURN_MIN_POWER = clamp(BiobuzzAutoDrive.TURN_MIN_POWER + direction * 0.01, 0.0, 1.0);
                break;
            case "TURN_MAX_POWER":
                BiobuzzAutoDrive.TURN_MAX_POWER = clamp(BiobuzzAutoDrive.TURN_MAX_POWER + direction * 0.05, 0.1, 1.0);
                break;
            case "TURN_TOLERANCE_DEG":
                BiobuzzAutoDrive.TURN_TOLERANCE_DEG = clamp(BiobuzzAutoDrive.TURN_TOLERANCE_DEG + direction * 0.25, 0.25, 10.0);
                break;
        }
    }

    private double gainValue(String name) {
        switch (name) {
            case "DRIVE_KP": return BiobuzzAutoDrive.DRIVE_KP;
            case "DRIVE_MIN_POWER": return BiobuzzAutoDrive.DRIVE_MIN_POWER;
            case "DRIVE_MAX_POWER": return BiobuzzAutoDrive.DRIVE_MAX_POWER;
            case "DRIVE_RAMP_SEC": return BiobuzzAutoDrive.DRIVE_RAMP_SEC;
            case "HOLD_KP": return BiobuzzAutoDrive.HOLD_KP;
            case "TURN_KP": return BiobuzzAutoDrive.TURN_KP;
            case "TURN_MIN_POWER": return BiobuzzAutoDrive.TURN_MIN_POWER;
            case "TURN_MAX_POWER": return BiobuzzAutoDrive.TURN_MAX_POWER;
            case "TURN_TOLERANCE_DEG": return BiobuzzAutoDrive.TURN_TOLERANCE_DEG;
            default: return 0.0;
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void showTelemetry(BiobuzzAutoDrive drive) {
        drive.update();
        telemetry.addData("Mode", mode + "   (B = next mode)");

        switch (mode) {
            case SCALARS:
                double reportedDist = Math.hypot(drive.getX(), drive.getY());
                double reportedAngle = Math.abs(drive.getHeading());
                telemetry.addLine("Push the robot a measured distance and/or turn it a measured angle");
                telemetry.addData("Reported distance", "%.2f in", reportedDist);
                telemetry.addData("Actual distance (D-pad Up/Down)", "%.0f in", actualDistance);
                telemetry.addData("Reported turn", "%.1f deg", reportedAngle);
                telemetry.addData("Actual turn (D-pad Left/Right)", "%.0f deg", actualAngle);
                if (reportedDist > 1.0) {
                    telemetry.addData("Try OTOS_LINEAR_SCALAR", "%.4f",
                            BiobuzzAutoDrive.OTOS_LINEAR_SCALAR * actualDistance / reportedDist);
                }
                if (reportedAngle > 5.0) {
                    telemetry.addData("Try OTOS_ANGULAR_SCALAR", "%.4f",
                            BiobuzzAutoDrive.OTOS_ANGULAR_SCALAR * actualAngle / reportedAngle);
                }
                telemetry.addData("Current scalars", "linear %.4f / angular %.4f",
                        BiobuzzAutoDrive.OTOS_LINEAR_SCALAR, BiobuzzAutoDrive.OTOS_ANGULAR_SCALAR);
                telemetry.addLine("Y = mark start | X = recalibrate (keep still)");
                break;

            case DRIVE:
                telemetry.addData("Test", "drive forward %.0f in (D-pad Up/Down)", testDistance);
                showGains(DRIVE_GAINS);
                telemetry.addLine("A = run | Bumpers = select gain | D-pad Left/Right = lower/raise");
                telemetry.addData("Last result", lastResult);
                break;

            case TURN:
                telemetry.addData("Test", "turn to %.0f deg (D-pad Up/Down)", testAngle);
                showGains(TURN_GAINS);
                telemetry.addLine("A = run | Bumpers = select gain | D-pad Left/Right = lower/raise");
                telemetry.addData("Last result", lastResult);
                break;
        }

        drive.addPoseTelemetry();
        telemetry.update();
    }

    private void showGains(String[] gains) {
        for (int i = 0; i < gains.length; i++) {
            telemetry.addData((i == gainIndex ? "> " : "  ") + gains[i], "%.4f", gainValue(gains[i]));
        }
    }
}
