package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Simple autonomous driving for the tank drive, using the SparkFun OTOS for position and heading.
 *
 * Field coordinates (inches, degrees):
 * - Origin is the field corner at the left end of the driver wall.
 * - +X runs to the right along the driver wall, +Y runs away from the drivers.
 * - Heading 0 faces +Y (away from the drivers). Heading INCREASES TURNING RIGHT (clockwise),
 *   so 90 faces +X. Headings are kept in the range -180 to 180.
 *
 * Example:
 * <pre>
 *   BiobuzzHardware robot = new BiobuzzHardware();
 *   robot.init(hardwareMap);
 *   BiobuzzAutoDrive drive = new BiobuzzAutoDrive(this, robot);
 *   drive.init();                      // configures and calibrates the OTOS - keep the robot still
 *   drive.setStartPose(12, 0, 0);      // x, y, heading where the robot sits at the start
 *   waitForStart();
 *   drive.driveForward(36);
 *   drive.turnToHeading(90);
 *   drive.driveBackward(12);
 * </pre>
 *
 * The OTOS must be mounted with its X axis to the robot's right and its Y axis forward (or the
 * mounting angle entered in OTOS_OFFSET_H). Offset, scalars and tuning values below are starting
 * values and need to be set and tuned once the sensor is on the robot.
 */
public class BiobuzzAutoDrive {

    // =========================================================
    //  OTOS SETUP  (set these when the sensor is mounted)
    // =========================================================
    /** Name of the SparkFun OTOS in the robot configuration. */
    public static final String OTOS_NAME = "otos";
    /** Sensor position relative to the robot's center of rotation. +X = right, +Y = forward (inches). */
    public static final double OTOS_OFFSET_X = 0.0;
    public static final double OTOS_OFFSET_Y = 0.0;
    /** Sensor mounting angle relative to the robot, degrees, + = counterclockwise. */
    public static final double OTOS_OFFSET_H = 0.0;
    /** Scale corrections from calibration (0.872 to 1.127). */
    public static final double OTOS_LINEAR_SCALAR = 1.0;
    public static final double OTOS_ANGULAR_SCALAR = 1.0;

    // =========================================================
    //  DRIVING (starting values - tune on the robot)
    // =========================================================
    public static final double DRIVE_MAX_POWER = 0.7;
    public static final double DRIVE_MIN_POWER = 0.12;
    /** Power per inch remaining; slows the robot down as it nears the target. */
    public static final double DRIVE_KP = 0.04;
    public static final double DRIVE_TOLERANCE_IN = 0.5;
    /** Time to ramp from the minimum to the maximum power at the start of a move. */
    public static final double DRIVE_RAMP_SEC = 0.3;
    /** Steering to hold the heading while driving: power per degree of error, and its limit. */
    public static final double HOLD_KP = 0.02;
    public static final double HOLD_MAX_CORR = 0.3;
    public static final double DEFAULT_DRIVE_TIMEOUT_S = 5.0;

    // =========================================================
    //  TURNING (starting values - tune on the robot)
    // =========================================================
    public static final double TURN_MAX_POWER = 0.6;
    public static final double TURN_MIN_POWER = 0.12;
    /** Power per degree of error. */
    public static final double TURN_KP = 0.012;
    public static final double TURN_TOLERANCE_DEG = 1.5;
    /** The heading must stay inside the tolerance this long before the turn counts as done. */
    public static final long TURN_SETTLE_MS = 150;
    public static final double DEFAULT_TURN_TIMEOUT_S = 3.0;

    private final LinearOpMode opMode;
    private final BiobuzzHardware robot;

    /** The OTOS, exposed so test OpModes can read the raw sensor values. */
    public SparkFunOTOS otos;

    // Where the robot started, in field coordinates
    private double startX = 0, startY = 0, startHeading = 0;

    // Current robot pose in field coordinates
    private double x = 0, y = 0, heading = 0;

    // Heading the robot holds while driving: the start heading, then the last turn target
    private double holdHeading = 0;

    private long lastTelemetryMs = 0;

    public BiobuzzAutoDrive(LinearOpMode opMode, BiobuzzHardware robot) {
        this.opMode = opMode;
        this.robot = robot;
    }

    /**
     * Configures and calibrates the OTOS. The robot must be completely still while this runs
     * (about a second).
     */
    public void init() {
        otos = opMode.hardwareMap.get(SparkFunOTOS.class, OTOS_NAME);
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setOffset(new SparkFunOTOS.Pose2D(OTOS_OFFSET_X, OTOS_OFFSET_Y, OTOS_OFFSET_H));
        otos.setLinearScalar(OTOS_LINEAR_SCALAR);
        otos.setAngularScalar(OTOS_ANGULAR_SCALAR);

        opMode.telemetry.addLine("Calibrating OTOS - do not move the robot");
        opMode.telemetry.update();
        otos.calibrateImu();
        otos.resetTracking();
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
        update();
    }

    /**
     * Sets where the robot is sitting at the start of the auto, in field coordinates.
     * Call this after init() and before the robot moves.
     */
    public void setStartPose(double xInches, double yInches, double headingDegrees) {
        startX = xInches;
        startY = yInches;
        startHeading = normalizeDegrees(headingDegrees);
        holdHeading = startHeading;
        // The OTOS now counts from zero in the robot's starting orientation.
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
        update();
    }

    /**
     * Reads the OTOS and updates the field pose. The OTOS reports x to the right, y forward and
     * heading counterclockwise, all relative to the robot's starting position and orientation.
     */
    public void update() {
        SparkFunOTOS.Pose2D p = otos.getPosition();
        double c = Math.cos(Math.toRadians(startHeading));
        double s = Math.sin(Math.toRadians(startHeading));
        x = startX + p.x * c + p.y * s;
        y = startY - p.x * s + p.y * c;
        heading = normalizeDegrees(startHeading - p.h);
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }

    /** Adds the current pose to telemetry (the caller calls telemetry.update()). */
    public void addPoseTelemetry() {
        opMode.telemetry.addData("X (in)", "%.1f", x);
        opMode.telemetry.addData("Y (in)", "%.1f", y);
        opMode.telemetry.addData("Heading (deg)", "%.1f", heading);
    }

    // =========================================================
    //  MOVES  (each returns true if it reached the target, false if it timed out)
    // =========================================================

    public boolean driveForward(double inches) {
        return driveDistance(Math.abs(inches), DEFAULT_DRIVE_TIMEOUT_S);
    }

    public boolean driveForward(double inches, double timeoutSeconds) {
        return driveDistance(Math.abs(inches), timeoutSeconds);
    }

    public boolean driveBackward(double inches) {
        return driveDistance(-Math.abs(inches), DEFAULT_DRIVE_TIMEOUT_S);
    }

    public boolean driveBackward(double inches, double timeoutSeconds) {
        return driveDistance(-Math.abs(inches), timeoutSeconds);
    }

    public boolean turnToHeading(double degrees) {
        return turn(degrees, DEFAULT_TURN_TIMEOUT_S);
    }

    public boolean turnToHeading(double degrees, double timeoutSeconds) {
        return turn(degrees, timeoutSeconds);
    }

    public void stop() {
        robot.setDrivePower(0, 0, 0, 0);
    }

    private boolean driveDistance(double signedInches, double timeoutSeconds) {
        update();
        double startPosX = x;
        double startPosY = y;
        double forwardX = Math.sin(Math.toRadians(holdHeading));
        double forwardY = Math.cos(Math.toRadians(holdHeading));
        double dir = signedInches < 0 ? -1.0 : 1.0;
        double target = Math.abs(signedInches);
        long startNs = System.nanoTime();
        boolean reached = false;

        while (opMode.opModeIsActive()) {
            update();
            double elapsed = (System.nanoTime() - startNs) / 1e9;

            // Distance covered along the direction being driven
            double traveled = ((x - startPosX) * forwardX + (y - startPosY) * forwardY) * dir;
            double remaining = target - traveled;

            if (remaining <= DRIVE_TOLERANCE_IN) {
                reached = true;
                break;
            }
            if (elapsed > timeoutSeconds) {
                break;
            }

            // Slow down near the target, and ramp up gently at the start
            double power = Math.max(DRIVE_MIN_POWER, Math.min(DRIVE_MAX_POWER, DRIVE_KP * remaining));
            double ramp = DRIVE_MIN_POWER + (DRIVE_MAX_POWER - DRIVE_MIN_POWER) * (elapsed / DRIVE_RAMP_SEC);
            power = Math.min(power, ramp);

            // Steer to hold the heading. A positive error means the robot must turn right.
            double error = normalizeDegrees(holdHeading - heading);
            double correction = Math.max(-HOLD_MAX_CORR, Math.min(HOLD_MAX_CORR, HOLD_KP * error));

            double base = dir * power;
            setSidePowers(base + correction, base - correction);

            showProgress("Driving", remaining);
            opMode.idle();
        }
        stop();
        return reached;
    }

    private boolean turn(double targetDegrees, double timeoutSeconds) {
        holdHeading = normalizeDegrees(targetDegrees);
        long startNs = System.nanoTime();
        long settleStartMs = -1;
        boolean reached = false;

        while (opMode.opModeIsActive()) {
            update();
            long nowMs = System.nanoTime() / 1000000L;
            double elapsed = (System.nanoTime() - startNs) / 1e9;

            // A positive error means the robot must turn right.
            double error = normalizeDegrees(holdHeading - heading);
            boolean inside = Math.abs(error) <= TURN_TOLERANCE_DEG;

            if (inside) {
                if (settleStartMs < 0) settleStartMs = nowMs;
                if (nowMs - settleStartMs >= TURN_SETTLE_MS) {
                    reached = true;
                    break;
                }
            } else {
                settleStartMs = -1;
            }
            if (elapsed > timeoutSeconds) {
                break;
            }

            double power = 0.0;
            if (!inside) {
                power = Math.max(TURN_MIN_POWER, Math.min(TURN_MAX_POWER, TURN_KP * Math.abs(error)));
                if (error < 0) power = -power;
            }
            // Turning right: left side forward, right side backward
            setSidePowers(power, -power);

            showProgress("Turning", error);
            opMode.idle();
        }
        stop();
        return reached;
    }

    /** Sends left/right power to the tank drive, scaling both down if either exceeds 1. */
    private void setSidePowers(double left, double right) {
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }
        robot.setDrivePower(left, left, right, right);
    }

    private void showProgress(String action, double value) {
        long nowMs = System.nanoTime() / 1000000L;
        if (nowMs - lastTelemetryMs < 100) return;
        lastTelemetryMs = nowMs;
        opMode.telemetry.addData(action, "%.1f left", value);
        addPoseTelemetry();
        opMode.telemetry.update();
    }

    /** Wraps an angle in degrees into the range -180 (inclusive) to 180 (exclusive). */
    public static double normalizeDegrees(double degrees) {
        double d = (degrees + 180.0) % 360.0;
        if (d < 0) d += 360.0;
        return d - 180.0;
    }
}
