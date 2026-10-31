package org.firstinspires.ftc.teamcode;

/**
 * BiobuzzHeadingHold
 * Non-blocking "turn to a field heading and hold it" controller for teleop. Call start() when a
 * button goes down, update() every loop while it is held (passing the current heading from
 * whatever sensor is used), and stop() when it is released. Send leftPower and rightPower to the drivetrain.
 *
 * The robot pivots in place until it is within AIM_TOLERANCE_DEG of the heading, then brakes. If it
 * is turned more than AIM_RELOCK_DEG off, it pivots back. It never drives.
 *
 * Headings use the field convention in BiobuzzAutoDrive: 0 faces +Y (away from the drivers), and
 * the heading increases turning right, so 90 faces +X.
 */
public class BiobuzzHeadingHold {

    private boolean active = false;
    private double targetHeading = 0;
    private boolean relocking = true;
    private boolean aimed = false;
    private long settleStartMs = -1;

    /** Power to send to the left and right sides of the drivetrain. */
    public double leftPower = 0;
    public double rightPower = 0;

    public void start(double headingDegrees) {
        targetHeading = BiobuzzAutoDrive.normalizeDegrees(headingDegrees);
        active = true;
        relocking = true;
        aimed = false;
        settleStartMs = -1;
        leftPower = 0;
        rightPower = 0;
    }

    public void stop() {
        active = false;
        leftPower = 0;
        rightPower = 0;
    }

    public boolean isActive() { return active; }

    /** True once the robot is on the heading and has settled. */
    public boolean isLocked() { return active && aimed; }

    public String getStatus() {
        if (!active) return "OFF";
        return String.format("%s %.0f deg", aimed ? "LOCKED" : "TURNING TO", targetHeading);
    }

    /** Call every loop while held, with the robot's current field heading in degrees. */
    public void update(double currentHeadingDegrees) {
        if (!active) return;
        long nowMs = System.nanoTime() / 1000000L;
        double error = BiobuzzAutoDrive.normalizeDegrees(targetHeading - currentHeadingDegrees);
        double absError = Math.abs(error);

        if (relocking) {
            if (absError <= BiobuzzConstants.AIM_TOLERANCE_DEG) {
                relocking = false;
                settleStartMs = nowMs;
            }
        } else if (absError > BiobuzzConstants.AIM_RELOCK_DEG) {
            relocking = true;
            aimed = false;
            settleStartMs = -1;
        }

        if (relocking) {
            pivot(error);
        } else {
            leftPower = 0;
            rightPower = 0;
            if (settleStartMs >= 0 && nowMs - settleStartMs >= BiobuzzConstants.AIM_SETTLE_MS) {
                aimed = true;
            }
        }
    }

    /** Turns in place toward zero error. A positive error means turn right: left forward, right back. */
    private void pivot(double error) {
        double power = Math.max(BiobuzzAutoDrive.TURN_MIN_POWER,
                Math.min(BiobuzzAutoDrive.TURN_MAX_POWER, BiobuzzAutoDrive.TURN_KP * Math.abs(error)));
        if (error < 0) power = -power;
        double max = Math.abs(power);
        leftPower = max > 1.0 ? power / max : power;
        rightPower = max > 1.0 ? -power / max : -power;
    }
}
