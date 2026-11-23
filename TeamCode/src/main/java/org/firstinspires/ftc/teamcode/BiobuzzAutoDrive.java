package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Simple autonomous driving for the tank drive, using the goBILDA Pinpoint for position and heading.
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
 *   drive.init();                      // configures and calibrates the Pinpoint - keep the robot still
 *   drive.setStartPose(12, 0, 0);      // x, y, heading where the robot sits at the start
 *   waitForStart();
 *   drive.driveForward(36);
 *   drive.turnToHeading(90);
 *   drive.driveBackward(12);
 * </pre>
 *
 * The Pinpoint reports x forward, y left and heading counterclockwise, relative to where it was
 * last zeroed. update() converts that into the field coordinates above. Pod offsets and encoder
 * directions are set below; the driving values are starting values to tune on the robot.
 */
public class BiobuzzAutoDrive {

    // =========================================================
    //  PINPOINT SETUP
    // =========================================================
    /** Name of the goBILDA Pinpoint in the robot configuration. */
    public static final String PINPOINT_NAME = "pinpoint";
    /**
     * Pod positions relative to the robot's center of rotation, in millimeters.
     * X pod (rolls forward/back): sideways distance, left = positive, right = negative.
     * Y pod (rolls sideways): forward distance, forward = positive, behind = negative.
     */
    public static final double PINPOINT_X_POD_OFFSET_MM = 0.0;
    public static final double PINPOINT_Y_POD_OFFSET_MM = 75.0;
    /** Pod count directions: X must increase pushing forward, Y must increase pushing left. */
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_X_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    /** Time to let the Pinpoint finish resetting and calibrating its IMU at init. */
    public static final long PINPOINT_RESET_WAIT_MS = 300;

    // =========================================================
    //  DRIVING (starting values - tune on the robot)
    // =========================================================
    public static double DRIVE_MAX_POWER = 0.7;
    public static double DRIVE_MIN_POWER = 0.12;
    /** Power per inch remaining; slows the robot down as it nears the target. */
    public static double DRIVE_KP = 0.020;
    public static double DRIVE_TOLERANCE_IN = 0.5;
    /** Time to ramp from the minimum to the maximum power at the start of a move. */
    public static double DRIVE_RAMP_SEC = 0.3;
    /** Steering to hold the heading while driving: power per degree of error, and its limit. */
    public static double HOLD_KP = 0.02;
    public static double HOLD_MAX_CORR = 0.3;
    public static final double DEFAULT_DRIVE_TIMEOUT_S = 5.0;

    // =========================================================
    //  TURNING (starting values - tune on the robot)
    // =========================================================
    public static double TURN_MAX_POWER = 0.6;
    public static double TURN_MIN_POWER = 0.12;
    /** Power per degree of error. */
    public static double TURN_KP = 0.012;
    public static double TURN_TOLERANCE_DEG = 1.5;
    /** The heading must stay inside the tolerance this long before the turn counts as done. */
    public static final long TURN_SETTLE_MS = 150;
    public static final double DEFAULT_TURN_TIMEOUT_S = 3.0;

    // =========================================================
    //  DRIVING TO A POINT  (driveTo / driveToBackward)
    // =========================================================
    /** If the robot is pointed more than this far from the point, it stops and turns in place first. */
    public static final double REAIM_TURN_DEG = 30.0;
    /** Inside this distance the aim is frozen, so the bearing to the point cannot flip around. */
    public static final double AIM_FREEZE_IN = 4.0;
    /** The robot is blocked if it moves less than this far in the stall time, while commanding power. */
    public static final double STALL_MIN_MOVE_IN = 1.0;
    public static final long STALL_TIME_MS = 1000;
    /** Stall detection is off this close to the point, where the robot is crawling at minimum power. */
    public static final double STALL_IGNORE_IN = 6.0;

    /**
     * Optional speed settings for a single move. Anything left out uses the defaults above
     * (DRIVE_MAX_POWER, DRIVE_RAMP_SEC, TURN_MAX_POWER), so the defaults can still be tuned in one place.
     *
     * <pre>
     *   drive.driveToBackward(47, 10, 8.0, BiobuzzAutoDrive.MoveSpeed.of(0.4));        // slower
     *   drive.driveToBackward(47, 10, 8.0, BiobuzzAutoDrive.MoveSpeed.of(0.4, 0.6));   // slower, gentler start
     * </pre>
     *
     * The ramp time applies to driving only; turns have no start ramp.
     */
    public static class MoveSpeed {
        private final double maxPower;
        private final double rampSec;

        private MoveSpeed(double maxPower, double rampSec) {
            this.maxPower = maxPower;
            this.rampSec = rampSec;
        }

        /** Top power for the move, 0 to 1. */
        public static MoveSpeed of(double maxPower) {
            return new MoveSpeed(maxPower, Double.NaN);
        }

        /** Top power for the move, 0 to 1, and the seconds to ramp up to it at the start. */
        public static MoveSpeed of(double maxPower, double rampSec) {
            return new MoveSpeed(maxPower, rampSec);
        }

        double power(double fallback, double minPower) {
            double p = Double.isNaN(maxPower) ? fallback : maxPower;
            return Math.max(minPower, Math.min(1.0, p));
        }

        double ramp(double fallback) {
            double r = Double.isNaN(rampSec) ? fallback : rampSec;
            return Math.max(0.01, r);
        }
    }

    private final LinearOpMode opMode;
    private final BiobuzzHardware robot;

    /** The Pinpoint, exposed so test OpModes can read the raw sensor values. */
    public GoBildaPinpointDriver pinpoint;

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
     * Configures and calibrates the Pinpoint. The robot must be completely still while this runs
     * (about a second).
     */
    public void init() {
        initWithoutCalibration();

        opMode.telemetry.addLine("Calibrating Pinpoint - do not move the robot");
        opMode.telemetry.update();
        pinpoint.resetPosAndIMU();   // zeroes the position and recalibrates the IMU
        opMode.sleep(PINPOINT_RESET_WAIT_MS);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        update();
    }

    /**
     * Configures the Pinpoint without calibrating the IMU or clearing its tracking. Use this in
     * teleop, where the robot may be moving or the sensor is already tracking. Call
     * setStartPose() afterward to set the pose.
     */
    public void initWithoutCalibration() {
        pinpoint = opMode.hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        pinpoint.setOffsets(PINPOINT_X_POD_OFFSET_MM, PINPOINT_Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(PINPOINT_X_DIRECTION, PINPOINT_Y_DIRECTION);
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
        // The Pinpoint now counts from zero in the robot's starting orientation.
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        update();
    }

    /**
     * Reads the Pinpoint and updates the field pose. The Pinpoint reports x forward, y left and
     * heading counterclockwise, all relative to the robot's starting position and orientation.
     * They are converted to right (px), forward (py) and counterclockwise (ph) before the
     * rotation into field coordinates.
     */
    public void update() {
        pinpoint.update();
        Pose2D p = pinpoint.getPosition();
        double px = -p.getY(DistanceUnit.INCH);
        double py = p.getX(DistanceUnit.INCH);
        double ph = p.getHeading(AngleUnit.DEGREES);

        double c = Math.cos(Math.toRadians(startHeading));
        double s = Math.sin(Math.toRadians(startHeading));
        x = startX + px * c + py * s;
        y = startY - px * s + py * c;
        heading = normalizeDegrees(startHeading - ph);
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
        return driveDistance(Math.abs(inches), DEFAULT_DRIVE_TIMEOUT_S, null);
    }

    public boolean driveForward(double inches, double timeoutSeconds) {
        return driveDistance(Math.abs(inches), timeoutSeconds, null);
    }

    public boolean driveForward(double inches, double timeoutSeconds, MoveSpeed speed) {
        return driveDistance(Math.abs(inches), timeoutSeconds, speed);
    }

    public boolean driveBackward(double inches) {
        return driveDistance(-Math.abs(inches), DEFAULT_DRIVE_TIMEOUT_S, null);
    }

    public boolean driveBackward(double inches, double timeoutSeconds) {
        return driveDistance(-Math.abs(inches), timeoutSeconds, null);
    }

    public boolean driveBackward(double inches, double timeoutSeconds, MoveSpeed speed) {
        return driveDistance(-Math.abs(inches), timeoutSeconds, speed);
    }

    public boolean turnToHeading(double degrees) {
        return turn(degrees, DEFAULT_TURN_TIMEOUT_S, null);
    }

    public boolean turnToHeading(double degrees, double timeoutSeconds) {
        return turn(degrees, timeoutSeconds, null);
    }

    public boolean turnToHeading(double degrees, double timeoutSeconds, MoveSpeed speed) {
        return turn(degrees, timeoutSeconds, speed);
    }

    /**
     * Drives to a point on the field. The robot turns in place if it is pointed far from the
     * point, then drives toward it, re-aiming from the live pose as it goes. If it is knocked off
     * course it arcs back toward the point. Returns false if it was blocked, timed out, or lost
     * its way; the caller can continue, and the next move aims from wherever the robot ended up.
     */
    public boolean driveTo(double xInches, double yInches) {
        return driveToPoint(xInches, yInches, false, DEFAULT_DRIVE_TIMEOUT_S, null);
    }

    public boolean driveTo(double xInches, double yInches, double timeoutSeconds) {
        return driveToPoint(xInches, yInches, false, timeoutSeconds, null);
    }

    public boolean driveTo(double xInches, double yInches, double timeoutSeconds, MoveSpeed speed) {
        return driveToPoint(xInches, yInches, false, timeoutSeconds, speed);
    }

    /** Same as driveTo, but the robot drives in reverse (its back faces the point). */
    public boolean driveToBackward(double xInches, double yInches) {
        return driveToPoint(xInches, yInches, true, DEFAULT_DRIVE_TIMEOUT_S, null);
    }

    public boolean driveToBackward(double xInches, double yInches, double timeoutSeconds) {
        return driveToPoint(xInches, yInches, true, timeoutSeconds, null);
    }

    public boolean driveToBackward(double xInches, double yInches, double timeoutSeconds, MoveSpeed speed) {
        return driveToPoint(xInches, yInches, true, timeoutSeconds, speed);
    }

    public void stop() {
        robot.setDrivePower(0, 0, 0, 0);
    }

    private boolean driveToPoint(double targetX, double targetY, boolean backward, double timeoutSeconds,
                                 MoveSpeed speed) {
        update();
        if (Math.hypot(targetX - x, targetY - y) <= DRIVE_TOLERANCE_IN) {
            return true;
        }

        double maxPower = speed == null ? DRIVE_MAX_POWER : speed.power(DRIVE_MAX_POWER, DRIVE_MIN_POWER);
        double rampSec = speed == null ? DRIVE_RAMP_SEC : speed.ramp(DRIVE_RAMP_SEC);

        long startNs = System.nanoTime();
        long rampStartNs = startNs;
        double desired = aimHeading(targetX, targetY, backward);
        double minDistance = Double.MAX_VALUE;
        double progressX = x;
        double progressY = y;
        long progressMs = startNs / 1000000L;
        boolean reached = false;
        String failure = "Drive TIMED OUT";

        while (opMode.opModeIsActive()) {
            update();
            long nowMs = System.nanoTime() / 1000000L;
            double elapsed = (System.nanoTime() - startNs) / 1e9;
            double distance = Math.hypot(targetX - x, targetY - y);

            if (distance <= DRIVE_TOLERANCE_IN) {
                reached = true;
                break;
            }
            // Closest approach: past the point and moving away from it, close enough to call it
            minDistance = Math.min(minDistance, distance);
            if (minDistance < 3.0 && distance > minDistance + 0.5) {
                reached = true;
                break;
            }
            if (elapsed > timeoutSeconds) {
                break;
            }

            // Re-aim from the live pose, except very near the point where the bearing is unstable
            if (distance > AIM_FREEZE_IN) {
                desired = aimHeading(targetX, targetY, backward);
            }
            holdHeading = desired;
            double error = normalizeDegrees(desired - heading);

            // Pointed far off (knocked hard, or a corner in the route): stop and turn in place
            if (distance > AIM_FREEZE_IN && Math.abs(error) > REAIM_TURN_DEG) {
                turn(desired, DEFAULT_TURN_TIMEOUT_S, null);
                rampStartNs = System.nanoTime();
                update();
                progressX = x;
                progressY = y;
                progressMs = System.nanoTime() / 1000000L;
                continue;
            }

            // Blocked: barely moved for a while although power is being sent
            if (distance > STALL_IGNORE_IN) {
                if (Math.hypot(x - progressX, y - progressY) >= STALL_MIN_MOVE_IN) {
                    progressX = x;
                    progressY = y;
                    progressMs = nowMs;
                } else if (nowMs - progressMs >= STALL_TIME_MS) {
                    failure = "Drive BLOCKED";
                    break;
                }
            }

            // Slow down near the point, and ramp up gently at the start
            double rampElapsed = (System.nanoTime() - rampStartNs) / 1e9;
            double power = Math.max(DRIVE_MIN_POWER, Math.min(maxPower, DRIVE_KP * distance));
            double ramp = DRIVE_MIN_POWER + (maxPower - DRIVE_MIN_POWER) * (rampElapsed / rampSec);
            power = Math.min(power, ramp);

            // A positive error means the robot must turn right
            double correction = Math.max(-HOLD_MAX_CORR, Math.min(HOLD_MAX_CORR, HOLD_KP * error));
            double base = backward ? -power : power;
            setSidePowers(base + correction, base - correction);

            showProgress("Driving to point", distance);
            opMode.idle();
        }
        stop();
        if (!reached && opMode.opModeIsActive()) {
            update();
            opMode.telemetry.log().add("%s, %.1f in from (%.0f, %.0f)", failure,
                    Math.hypot(targetX - x, targetY - y), targetX, targetY);
            opMode.telemetry.update();
        }
        return reached;
    }

    /**
     * Heading the robot should have to travel to the point. Heading 0 faces +Y and 90 faces +X, so
     * the bearing from the robot to the point is atan2(dx, dy). In reverse the back faces the point.
     */
    private double aimHeading(double targetX, double targetY, boolean backward) {
        double bearing = Math.toDegrees(Math.atan2(targetX - x, targetY - y));
        return normalizeDegrees(backward ? bearing + 180.0 : bearing);
    }

    private boolean driveDistance(double signedInches, double timeoutSeconds, MoveSpeed speed) {
        double maxPower = speed == null ? DRIVE_MAX_POWER : speed.power(DRIVE_MAX_POWER, DRIVE_MIN_POWER);
        double rampSec = speed == null ? DRIVE_RAMP_SEC : speed.ramp(DRIVE_RAMP_SEC);
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
            double power = Math.max(DRIVE_MIN_POWER, Math.min(maxPower, DRIVE_KP * remaining));
            double ramp = DRIVE_MIN_POWER + (maxPower - DRIVE_MIN_POWER) * (elapsed / rampSec);
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
        if (!reached && opMode.opModeIsActive()) {
            opMode.telemetry.log().add("Drive TIMED OUT, %.1f in short", Math.max(0.0, target - ((x - startPosX) * forwardX + (y - startPosY) * forwardY) * dir));
            opMode.telemetry.update();
        }
        return reached;
    }

    private boolean turn(double targetDegrees, double timeoutSeconds, MoveSpeed speed) {
        double maxPower = speed == null ? TURN_MAX_POWER : speed.power(TURN_MAX_POWER, TURN_MIN_POWER);
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
                power = Math.max(TURN_MIN_POWER, Math.min(maxPower, TURN_KP * Math.abs(error)));
                if (error < 0) power = -power;
            }
            // Turning right: left side forward, right side backward
            setSidePowers(power, -power);

            showProgress("Turning", error);
            opMode.idle();
        }
        stop();
        if (!reached && opMode.opModeIsActive()) {
            opMode.telemetry.log().add("Turn TIMED OUT, %.1f deg off", Math.abs(normalizeDegrees(holdHeading - heading)));
            opMode.telemetry.update();
        }
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
