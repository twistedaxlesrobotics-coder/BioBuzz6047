package org.firstinspires.ftc.teamcode;

/**
 * BiobuzzFieldDrive
 * Non-blocking field-centric drive controller for teleop, with two versions. In both, the robot turns
 * to a heading on the field and holds it, and a stick or trigger sets how fast it drives along that
 * heading. Headings are chosen with the sticks.
 *
 * POINT AND GO (update):
 * - RIGHT stick: the FRONT of the robot points the way the stick points; the trigger drives forward.
 * - LEFT stick: the BACK of the robot points the way the stick points; the trigger drives backward.
 * - If both sticks are pushed, the right stick wins.
 * - No stick pushed: the robot holds its last heading and does not move.
 * - Pushing a stick starts the robot at FD_THROTTLE_FLOOR power; the trigger adds the rest, up to
 *   100%. Power is scaled down while the robot is turned away from the target heading, so it turns
 *   first and then rolls.
 *
 * HEADING + DRIVE (updateHeadingArcade):
 * - RIGHT stick: the FRONT of the robot points the way the stick points. It only turns; it does not
 *   move the robot. Released, the robot holds the last heading.
 * - LEFT stick: forward or back along the robot's heading, proportional, topping out at
 *   FD_ARCADE_MAX_POWER. The left trigger raises that top power toward 100%.
 * - The two sticks are independent, so the robot may drive and turn at once.
 *
 * Call one of the update methods every loop (after the pose has been updated) with the raw stick and
 * trigger values and the current field heading, then send leftPower and rightPower to the
 * drivetrain. Call holdHeadingNow() whenever something else is driving, the pose was reset, or the
 * version was switched, so the robot does not snap back to an old heading.
 *
 * The target heading is worked out separately from the turning and ramping code, so another source of
 * target heading (such as aiming at a field point) can be added without touching them.
 *
 * Headings use the field convention in BiobuzzAutoDrive: 0 faces +Y (away from the drivers), and the
 * heading increases turning right, so 90 faces +X.
 */
public class BiobuzzFieldDrive {

    /** Power to send to the left and right sides of the drivetrain. */
    public double leftPower = 0;
    public double rightPower = 0;

    // Values for telemetry
    public double targetHeading = 0;
    public double headingError = 0;
    /** Turn rate in degrees per second, positive turning right. */
    public double turnRateDps = 0;
    /** Signed power asked for before the ramp (positive = forward). */
    public double throttle = 0;
    public double alignScale = 0;
    /** Signed forward/back power after the ramp. */
    public double drivePower = 0;
    public boolean targetFrozen = false;
    public String activeStick = "NONE";

    private boolean hasTarget = false;
    private double prevRightMag = 0;
    private double prevLeftMag = 0;
    private double lastHeading = 0;
    private boolean hasLastHeading = false;
    private double turnOut = 0;

    /**
     * Holds the given heading and restarts the ramps from zero. Use it while another controller is
     * driving, after a pose reset, and when switching versions.
     */
    public void holdHeadingNow(double heading) {
        targetHeading = BiobuzzAutoDrive.normalizeDegrees(heading);
        hasTarget = true;
        drivePower = 0;
        turnOut = 0;
        leftPower = 0;
        rightPower = 0;
        lastHeading = heading;
        hasLastHeading = true;
        turnRateDps = 0;
        activeStick = "NONE";
        targetFrozen = false;
    }

    // =========================================================
    //  POINT AND GO
    // =========================================================

    public void update(double leftStickX, double leftStickY, double rightStickX, double rightStickY,
                       double trigger, double heading, double dtSeconds) {
        double dt = clampDt(dtSeconds);
        prepare(heading, dt);

        // --- Target heading, and which stick is in charge ---
        int stick = updateTargetFromSticks(leftStickX, leftStickY, rightStickX, rightStickY);
        boolean stickHeld = stick != 0;

        headingError = BiobuzzAutoDrive.normalizeDegrees(targetHeading - heading);
        double absError = Math.abs(headingError);

        // --- Alignment: no forward power while turned far off, full power when close ---
        double cutoff = Math.max(BiobuzzConstants.FD_ALIGN_CUTOFF_DEG,
                BiobuzzConstants.FD_ALIGN_FULL_DEG + 0.1);
        double full = BiobuzzConstants.FD_ALIGN_FULL_DEG;
        if (absError >= cutoff) {
            alignScale = 0.0;
        } else if (absError <= full) {
            alignScale = 1.0;
        } else {
            alignScale = (cutoff - absError) / (cutoff - full);
        }

        // --- Throttle: floor plus the trigger, only while a stick is held ---
        double wanted = 0.0;
        if (stickHeld) {
            double t = Math.max(0.0, Math.min(1.0, trigger));
            double floor = BiobuzzConstants.FD_THROTTLE_FLOOR;
            double power = floor + (1.0 - floor) * t;
            wanted = stick * power * alignScale;   // stick is +1 (right, forward) or -1 (left, back)
        }
        throttle = wanted;

        rampDrive(wanted, dt);
        turnAndCombine(dt);
    }

    // =========================================================
    //  HEADING + DRIVE
    // =========================================================

    /**
     * Right stick picks the heading (no movement), left stick drives forward/back, left trigger
     * raises the top power. Pass the raw gamepad values (stick up is negative y).
     */
    public void updateHeadingArcade(double rightStickX, double rightStickY, double leftStickY,
                                    double trigger, double heading, double dtSeconds) {
        double dt = clampDt(dtSeconds);
        prepare(heading, dt);

        // --- Target heading from the right stick only; released, it holds the last heading ---
        double rightMag = Math.hypot(rightStickX, rightStickY);
        targetFrozen = false;
        if (rightMag >= BiobuzzConstants.FD_STICK_DEADBAND) {
            targetFrozen = isReleasing(rightMag, prevRightMag);
            if (!targetFrozen) {
                targetHeading = stickHeading(rightStickX, rightStickY);
            }
            activeStick = "RIGHT (heading)";
        } else {
            activeStick = "NONE (holding)";
        }
        prevRightMag = rightMag;

        headingError = BiobuzzAutoDrive.normalizeDegrees(targetHeading - heading);
        alignScale = 1.0;   // the sticks are independent, so driving is never cut for being off heading

        // --- Speed from the left stick, topping out at ARCADE_MAX, raised toward 100% by the trigger ---
        double stick = -leftStickY;
        if (Math.abs(stick) < BiobuzzConstants.FD_ARCADE_DEADBAND) {
            stick = 0.0;
        }
        double base = Math.max(0.0, Math.min(1.0, BiobuzzConstants.FD_ARCADE_MAX_POWER));
        double t = Math.max(0.0, Math.min(1.0, trigger));
        double top = base + (1.0 - base) * t;
        double wanted = Math.max(-1.0, Math.min(1.0, stick * top));
        throttle = wanted;

        rampDrive(wanted, dt);
        turnAndCombine(dt);
    }

    // =========================================================
    //  SHARED
    // =========================================================

    private static double clampDt(double dtSeconds) {
        return Math.max(0.001, Math.min(0.1, dtSeconds));
    }

    /** Starts holding the current heading if there is no target yet, and works out the turn rate. */
    private void prepare(double heading, double dt) {
        if (!hasTarget) {
            holdHeadingNow(heading);
        }

        // Turn rate worked out from the heading, smoothed
        if (hasLastHeading) {
            double raw = BiobuzzAutoDrive.normalizeDegrees(heading - lastHeading) / dt;
            turnRateDps += BiobuzzConstants.FD_RATE_FILTER * (raw - turnRateDps);
        }
        lastHeading = heading;
        hasLastHeading = true;
    }

    /** Forward/back ramp, gentler when slowing down. */
    private void rampDrive(double wanted, double dt) {
        boolean speedingUp = wanted * drivePower >= 0 && Math.abs(wanted) >= Math.abs(drivePower);
        double rate = speedingUp ? BiobuzzConstants.FD_ACCEL_RATE : BiobuzzConstants.FD_DECEL_RATE;
        drivePower = BiobuzzConstants.slew(drivePower, wanted, rate * dt);
    }

    /** Turns toward the target heading (a positive error means turn right) and combines with the drive. */
    private void turnAndCombine(double dt) {
        double absError = Math.abs(headingError);

        double turn = 0.0;
        if (absError > BiobuzzConstants.FD_TURN_TOLERANCE_DEG) {
            double base = BiobuzzConstants.FD_TURN_KP * headingError;
            if (Math.abs(base) < BiobuzzConstants.FD_TURN_MIN_POWER) {
                base = Math.signum(headingError) * BiobuzzConstants.FD_TURN_MIN_POWER;
            }
            turn = base - BiobuzzConstants.FD_TURN_KD * turnRateDps;
            double max = BiobuzzConstants.FD_TURN_MAX_POWER;
            turn = Math.max(-max, Math.min(max, turn));
        }

        // Soften the start of a turn. Anything that lowers turning power (braking, reversing) is
        // applied at once, so the damping is never delayed.
        double rampRate = BiobuzzConstants.FD_TURN_RAMP_RATE;
        if (rampRate > 0 && turn * turnOut >= 0 && Math.abs(turn) > Math.abs(turnOut)) {
            turnOut = BiobuzzConstants.slew(turnOut, turn, rampRate * dt);
        } else {
            turnOut = turn;
        }

        // Combine, normalizing if either side exceeds 1.0
        double left = drivePower + turnOut;
        double right = drivePower - turnOut;
        double biggest = Math.max(Math.abs(left), Math.abs(right));
        if (biggest > 1.0) {
            left /= biggest;
            right /= biggest;
        }
        leftPower = left;
        rightPower = right;
    }

    /**
     * Updates the target heading from the sticks (point and go). Returns +1 if the right stick
     * (front) is in charge, -1 if the left stick (back) is, or 0 if neither is pushed past the
     * deadband.
     */
    private int updateTargetFromSticks(double leftX, double leftY, double rightX, double rightY) {
        double rightMag = Math.hypot(rightX, rightY);
        double leftMag = Math.hypot(leftX, leftY);
        boolean rightActive = rightMag >= BiobuzzConstants.FD_STICK_DEADBAND;
        boolean leftActive = !rightActive && leftMag >= BiobuzzConstants.FD_STICK_DEADBAND;

        int result = 0;
        targetFrozen = false;
        if (rightActive) {
            targetFrozen = isReleasing(rightMag, prevRightMag);
            if (!targetFrozen) {
                targetHeading = stickHeading(rightX, rightY);
            }
            activeStick = "RIGHT (front)";
            result = 1;
        } else if (leftActive) {
            targetFrozen = isReleasing(leftMag, prevLeftMag);
            if (!targetFrozen) {
                // The back points where the stick points, so the front faces the other way
                targetHeading = BiobuzzAutoDrive.normalizeDegrees(stickHeading(leftX, leftY) + 180.0);
            }
            activeStick = "LEFT (back)";
            result = -1;
        } else {
            activeStick = "NONE";
        }

        prevRightMag = rightMag;
        prevLeftMag = leftMag;
        return result;
    }

    /** True while a stick is on its way back to center and below the freeze level. */
    private static boolean isReleasing(double magnitude, double previousMagnitude) {
        return magnitude < previousMagnitude - 0.005 && magnitude < BiobuzzConstants.FD_STICK_FREEZE;
    }

    /**
     * Field heading for a stick position, from the raw gamepad values (stick up is negative y).
     * Up is 0 (away from the drivers), right is 90.
     */
    private static double stickHeading(double x, double y) {
        return BiobuzzAutoDrive.normalizeDegrees(Math.toDegrees(Math.atan2(x, -y)));
    }
}
