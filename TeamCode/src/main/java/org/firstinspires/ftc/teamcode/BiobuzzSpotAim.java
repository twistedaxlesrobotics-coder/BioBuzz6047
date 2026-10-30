package org.firstinspires.ftc.teamcode;

/**
 * BiobuzzSpotAim
 * Non-blocking "go to a spot and aim at the goal" controller for teleop. Call start() when a
 * button goes down, update() every loop while it is held (after the pose has been updated), and
 * stop() when it is released. Send leftPower and rightPower to the drivetrain.
 *
 * Phases:
 * - APPROACH: drive to the spot, forward or in reverse, whichever needs less turning. Turns in
 *   place first if pointed far off, and recovers if knocked. Gives up if the robot is blocked.
 * - AIM: pivot until the back of the robot (the shooter) faces the goal, then hold that aim. The
 *   aim is recomputed from the live pose, so if the robot is pushed it keeps facing the goal from
 *   wherever it is. It never drives back to the spot.
 * - BLOCKED: the robot could not move toward the spot. Power is zero (brakes) until stop()/start().
 *
 * Field coordinates and heading convention are described in BiobuzzAutoDrive.
 */
public class BiobuzzSpotAim {

    private enum Phase { APPROACH, AIM, BLOCKED }

    private final BiobuzzAutoDrive pose;

    private boolean active = false;
    private Phase phase = Phase.APPROACH;

    private double spotX, spotY, goalX, goalY;

    // Approach state
    private boolean backward = false;
    private boolean pivoting = false;
    private double frozenDesired = 0;
    private double minDistance = Double.MAX_VALUE;
    private long rampStartNs = 0;
    private double progressX = 0, progressY = 0;
    private long progressMs = 0;

    // Aim state
    private boolean relocking = true;
    private boolean aimed = false;
    private long settleStartMs = -1;

    /** Power to send to the left and right sides of the drivetrain. */
    public double leftPower = 0;
    public double rightPower = 0;

    public BiobuzzSpotAim(BiobuzzAutoDrive pose) {
        this.pose = pose;
    }

    public void start(double spotX, double spotY, double goalX, double goalY) {
        this.spotX = spotX;
        this.spotY = spotY;
        this.goalX = goalX;
        this.goalY = goalY;

        double x = pose.getX();
        double y = pose.getY();
        double h = pose.getHeading();
        long nowMs = System.nanoTime() / 1000000L;

        active = true;
        phase = Phase.APPROACH;
        aimed = false;
        relocking = true;
        settleStartMs = -1;
        pivoting = false;
        minDistance = Double.MAX_VALUE;
        rampStartNs = System.nanoTime();
        progressX = x;
        progressY = y;
        progressMs = nowMs;

        // Pick forward or reverse, whichever needs the smaller turn to face the spot
        double bearing = bearingTo(spotX, spotY);
        double forwardError = Math.abs(BiobuzzAutoDrive.normalizeDegrees(bearing - h));
        double reverseError = Math.abs(BiobuzzAutoDrive.normalizeDegrees(bearing + 180.0 - h));
        backward = reverseError < forwardError;
        frozenDesired = BiobuzzAutoDrive.normalizeDegrees(backward ? bearing + 180.0 : bearing);

        leftPower = 0;
        rightPower = 0;
    }

    public void stop() {
        active = false;
        leftPower = 0;
        rightPower = 0;
    }

    public boolean isActive() { return active; }

    /** True once the robot is aimed at the goal and has settled. */
    public boolean isLocked() { return active && phase == Phase.AIM && aimed; }

    public String getStatus() {
        if (!active) return "OFF";
        switch (phase) {
            case APPROACH: return pivoting ? "TURNING TO SPOT" : "DRIVING TO SPOT";
            case AIM: return aimed ? "LOCKED" : "AIMING";
            default: return "BLOCKED (release to reset)";
        }
    }

    public void update() {
        if (!active) return;
        long nowMs = System.nanoTime() / 1000000L;
        switch (phase) {
            case APPROACH: approach(nowMs); break;
            case AIM: aim(nowMs); break;
            default: leftPower = 0; rightPower = 0; break;
        }
    }

    private void approach(long nowMs) {
        double x = pose.getX();
        double y = pose.getY();
        double h = pose.getHeading();
        double distance = Math.hypot(spotX - x, spotY - y);

        minDistance = Math.min(minDistance, distance);
        boolean arrived = distance <= BiobuzzConstants.SPOT_ARRIVE_TOL_IN;
        // Closest approach: past the spot and moving away from it, close enough to call it
        boolean passed = minDistance < 3.0 && distance > minDistance + 0.5;
        if (arrived || passed) {
            phase = Phase.AIM;
            relocking = true;
            aimed = false;
            settleStartMs = -1;
            aim(nowMs);
            return;
        }

        // Re-aim from the live pose, except very near the spot where the bearing is unstable
        double bearing = bearingTo(spotX, spotY);
        if (distance > BiobuzzAutoDrive.AIM_FREEZE_IN) {
            frozenDesired = BiobuzzAutoDrive.normalizeDegrees(backward ? bearing + 180.0 : bearing);
        }
        double error = BiobuzzAutoDrive.normalizeDegrees(frozenDesired - h);

        // Pointed far off: stop and turn in place first, then drive
        if (distance > BiobuzzAutoDrive.AIM_FREEZE_IN) {
            if (!pivoting && Math.abs(error) > BiobuzzAutoDrive.REAIM_TURN_DEG) {
                pivoting = true;
            } else if (pivoting && Math.abs(error) <= BiobuzzAutoDrive.TURN_TOLERANCE_DEG) {
                pivoting = false;
                rampStartNs = System.nanoTime();
            }
        }
        if (pivoting) {
            progressX = x;
            progressY = y;
            progressMs = nowMs;
            pivot(error);
            return;
        }

        // Blocked: barely moved for a while although power is being sent
        if (distance > BiobuzzAutoDrive.STALL_IGNORE_IN) {
            if (Math.hypot(x - progressX, y - progressY) >= BiobuzzAutoDrive.STALL_MIN_MOVE_IN) {
                progressX = x;
                progressY = y;
                progressMs = nowMs;
            } else if (nowMs - progressMs >= BiobuzzAutoDrive.STALL_TIME_MS) {
                phase = Phase.BLOCKED;
                leftPower = 0;
                rightPower = 0;
                return;
            }
        }

        // Slow down near the spot, and ramp up gently at the start
        double maxPower = Math.max(BiobuzzAutoDrive.DRIVE_MIN_POWER,
                Math.min(BiobuzzConstants.SPOT_APPROACH_MAX_POWER, BiobuzzAutoDrive.DRIVE_MAX_POWER));
        double rampElapsed = (System.nanoTime() - rampStartNs) / 1e9;
        double power = Math.max(BiobuzzAutoDrive.DRIVE_MIN_POWER,
                Math.min(maxPower, BiobuzzAutoDrive.DRIVE_KP * distance));
        double ramp = BiobuzzAutoDrive.DRIVE_MIN_POWER
                + (maxPower - BiobuzzAutoDrive.DRIVE_MIN_POWER)
                * (rampElapsed / Math.max(0.01, BiobuzzAutoDrive.DRIVE_RAMP_SEC));
        power = Math.min(power, ramp);

        // A positive error means the robot must turn right
        double correction = Math.max(-BiobuzzAutoDrive.HOLD_MAX_CORR,
                Math.min(BiobuzzAutoDrive.HOLD_MAX_CORR, BiobuzzAutoDrive.HOLD_KP * error));
        double base = backward ? -power : power;
        setSides(base + correction, base - correction);
    }

    private void aim(long nowMs) {
        double h = pose.getHeading();
        // The shooter is on the back of the robot, so the back must face the goal
        double bearing = bearingTo(goalX, goalY);
        double error = BiobuzzAutoDrive.normalizeDegrees(bearing + 180.0 - h);
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
        setSides(power, -power);
    }

    /** Heading 0 faces +Y and 90 faces +X, so the bearing to a point is atan2(dx, dy). */
    private double bearingTo(double targetX, double targetY) {
        return Math.toDegrees(Math.atan2(targetX - pose.getX(), targetY - pose.getY()));
    }

    private void setSides(double left, double right) {
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }
        leftPower = left;
        rightPower = right;
    }
}
