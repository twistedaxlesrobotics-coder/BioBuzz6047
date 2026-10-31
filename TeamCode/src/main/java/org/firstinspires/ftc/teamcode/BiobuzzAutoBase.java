package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * BiobuzzAutoBase
 * Shared setup and shooting for the autonomous OpModes. A subclass supplies its start pose and the
 * route it drives after the opening shot, and adds the @Autonomous annotation.
 *
 * Sequence: init and calibrate the OTOS, set the start pose, then on START: spool the shooter while
 * backing up, wait for the wheel to reach speed, shoot, run the subclass route. The pose is saved
 * for teleop when the auto ends, even if it is stopped early.
 *
 * Field coordinates and heading convention are described in BiobuzzAutoDrive.
 */
public abstract class BiobuzzAutoBase extends LinearOpMode {

    protected final BiobuzzHardware robot = new BiobuzzHardware();
    protected BiobuzzAutoDrive drive;

    /** Time since the auto started (reset on START). */
    protected final ElapsedTime autoTimer = new ElapsedTime();

    /** Start pose: center of the robot, in inches and degrees. */
    protected abstract double getStartX();
    protected abstract double getStartY();
    protected abstract double getStartHeading();

    /** Where the robot backs up to for the opening shot: center of the robot, in inches. */
    protected abstract double getShotX();
    protected abstract double getShotY();

    /**
     * Speed for the opening move to the shot spot. Null uses the defaults; an auto can override this
     * to run that move slower.
     */
    protected BiobuzzAutoDrive.MoveSpeed getMoveSpeed() {
        return null;
    }

    /**
     * Seconds after START before the opening shot may fire. The robot still drives to the shot spot
     * and spins the shooter up straight away, then waits here. An auto can override this to delay
     * its shot.
     */
    protected double getEarliestShotSeconds() {
        return 0.0;
    }

    /** The moves that follow the opening shot. */
    protected abstract void runRoute();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        drive = new BiobuzzAutoDrive(this, robot);
        drive.init();   // calibrates the OTOS - keep the robot still
        drive.setStartPose(getStartX(), getStartY(), getStartHeading());

        telemetry.addData("Status", "Ready. Press START.");
        drive.addPoseTelemetry();
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;
        autoTimer.reset();

        try {
            openingShot();
            runRoute();
        } finally {
            drive.stop();
            robot.setShooterWheelRpm(0);

            // Save the final pose for teleop (also runs if the auto is stopped early)
            drive.update();
            PoseStore.save(drive.getX(), drive.getY(), drive.getHeading());
        }
    }

    /**
     * Spools the shooter while backing up to the shot spot, waits for the wheel to reach speed,
     * then shoots. If the robot is blocked or knocked off, it tries to reach the spot once more
     * and then shoots from wherever it is.
     */
    protected void openingShot() {
        spoolShooter();
        if (!drive.driveToBackward(getShotX(), getShotY(),
                BiobuzzAutoDrive.DEFAULT_DRIVE_TIMEOUT_S, getMoveSpeed())) {
            telemetry.log().add("Shot spot missed, trying once more");
            drive.driveToBackward(getShotX(), getShotY(),
                    BiobuzzAutoDrive.DEFAULT_DRIVE_TIMEOUT_S, getMoveSpeed());
        }

        // Hold the shot until the earliest allowed time, with the shooter spinning
        waitUntilAutoTime(getEarliestShotSeconds());

        // Running late: skip the shot so there is time to park
        if (pastParkDeadline()) {
            telemetry.log().add("Past park deadline, skipping the shot");
            robot.setShooterWheelRpm(0);
            return;
        }
        waitForShooterAtSpeed();
        shoot();
    }

    /** Waits, keeping the shooter at speed, until the auto timer reaches the given number of seconds. */
    protected void waitUntilAutoTime(double seconds) {
        while (opModeIsActive() && autoTimer.seconds() < seconds) {
            robot.applyShooterPIDFWithVoltageComp();
            telemetry.addData("Waiting to shoot", "%.1f s left", seconds - autoTimer.seconds());
            telemetry.addData("Shooter RPM", "%.0f", robot.getShooterWheelRpm());
            telemetry.update();
            idle();
        }
    }

    /** True once the auto has run long enough that only the park should still be attempted. */
    protected boolean pastParkDeadline() {
        return autoTimer.seconds() > BiobuzzConstants.AUTO_PARK_DEADLINE_S;
    }

    /**
     * Backs through an intermediate waypoint, unless the auto is past the park deadline, in which
     * case it is skipped. Returns true if the waypoint was reached.
     */
    protected boolean viaBackward(double x, double y, double timeoutSeconds) {
        return viaBackward(x, y, timeoutSeconds, null);
    }

    protected boolean viaBackward(double x, double y, double timeoutSeconds,
                                  BiobuzzAutoDrive.MoveSpeed speed) {
        if (pastParkDeadline()) {
            telemetry.log().add("Past park deadline, skipping waypoint (%.0f, %.0f)", x, y);
            return false;
        }
        return drive.driveToBackward(x, y, timeoutSeconds, speed);
    }

    /**
     * Backs to the park point and keeps trying until it gets there or the auto ends. After a
     * blocked or timed-out attempt it pauses briefly, then aims again from where it is.
     */
    protected void parkAt(double x, double y, double timeoutSeconds) {
        parkAt(x, y, timeoutSeconds, null);
    }

    protected void parkAt(double x, double y, double timeoutSeconds, BiobuzzAutoDrive.MoveSpeed speed) {
        while (opModeIsActive()) {
            if (drive.driveToBackward(x, y, timeoutSeconds, speed)) {
                return;
            }
            telemetry.log().add("Park not reached, trying again");
            sleep(BiobuzzConstants.AUTO_PARK_RETRY_PAUSE_MS);
        }
    }

    protected void spoolShooter() {
        robot.setShooterWheelRpm(BiobuzzConstants.RPM_CLOSE);
        robot.applyShooterPIDFWithVoltageComp();
    }

    /** Opens the gate, waits, then stops the shooter after a short pause. */
    protected void shoot() {
        robot.setGate(true);
        sleep(BiobuzzConstants.AUTO_GATE_OPEN_MS);
        robot.setGate(false);

        sleep(BiobuzzConstants.AUTO_AFTER_SHOT_WAIT_MS);
        robot.setShooterWheelRpm(0);
    }

    /** Waits for the shooter wheel to hold its target RPM, giving up after a timeout. */
    protected void waitForShooterAtSpeed() {
        long startNs = System.nanoTime();
        long stableSinceMs = -1;

        while (opModeIsActive()) {
            long nowMs = System.nanoTime() / 1000000L;
            double elapsed = (System.nanoTime() - startNs) / 1e9;

            if (robot.isShooterAtSpeed(BiobuzzConstants.RPM_CLOSE, BiobuzzConstants.SHOOTER_RPM_TOLERANCE)) {
                if (stableSinceMs < 0) stableSinceMs = nowMs;
                if (nowMs - stableSinceMs >= BiobuzzConstants.AUTO_SPINUP_STABLE_MS) return;
            } else {
                stableSinceMs = -1;
            }
            if (elapsed > BiobuzzConstants.AUTO_SPINUP_TIMEOUT_S) return;

            robot.applyShooterPIDFWithVoltageComp();
            telemetry.addData("Shooter RPM", "%.0f", robot.getShooterWheelRpm());
            telemetry.update();
            idle();
        }
    }
}
