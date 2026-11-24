package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * BiobuzzConstants
 * Stores constants for shooter presets and tolerances, ported from last year's Decode logic.
 */
public class BiobuzzConstants {

    // =========================================================
    //  SHOOTER (SPEEDS + TOLERANCES)
    // ================ =========================================

    public enum ShooterPreset { STOP, CLOSE, }

    // Preset RPM values (mimicking last year's top motor speeds)
    public static final double RPM_CLOSE = 2350.0;

    /**
     *
     */
    public static double getTargetRpm(ShooterPreset preset) {
        switch (preset) {
            case CLOSE: return RPM_CLOSE;
            default:    return 0.0;
        }
    }

    /** How close (in RPM) the wheel must be to its target to count as "at speed". */
    public static final double SHOOTER_RPM_TOLERANCE = 75.0;

    // =========================================================
    //  AUTO SHOOTING
    // =========================================================

    /** After this many seconds into the auto, skip any shot or waypoint not yet done and go park. */
    public static final double AUTO_PARK_DEADLINE_S = 22.0;

    /** Pause between park attempts when the robot was blocked or knocked off. */
    public static final long AUTO_PARK_RETRY_PAUSE_MS = 300;

    /** The wheel must stay within tolerance this long before the gate opens. */
    public static final long AUTO_SPINUP_STABLE_MS = 200;

    /** Give up waiting for the wheel after this long, and shoot anyway. */
    public static final double AUTO_SPINUP_TIMEOUT_S = 3.0;

    /** How long the gate stays open for the shot. */
    public static final long AUTO_GATE_OPEN_MS = 1000;

    /** Pause after the gate closes before the shooter is stopped. */
    public static final long AUTO_AFTER_SHOT_WAIT_MS = 1000;

    // =========================================================
    //  TELEOP DRIVING (Gamepad 1 sticks)
    // =========================================================

    /** Full left-stick forward/back is this fraction of full power. Lower it to slow the robot down. */
    public static final double TELEOP_DRIVE_SCALE = 0.7;

    /** Full right-stick turn is this fraction of full power. Lower it to turn more gently. */
    public static final double TELEOP_TURN_SCALE = 0.4;

    /** Straight-drive gyro assist: active while driving with the right stick inside this deadband. */
    public static final double STRAIGHT_ASSIST_DEADBAND = 0.08;

    /** Steering power added per degree the robot has turned off its heading, and its limit. */
    public static final double STRAIGHT_ASSIST_KP = 0.02;

    /** After a turn, the assist locks on only once the robot rotates slower than this (degrees per second). */
    public static final double STRAIGHT_ASSIST_SETTLE_DPS = 20.0;
    public static final double STRAIGHT_ASSIST_MAX = 0.20;

    // =========================================================
    //  TELEOP AIM AT GOAL  (Gamepad 1 bumpers)
    //  Positions are field coordinates in inches. These are estimates - update after measuring.
    //  Teleop only uses the GOAL points (the bumpers aim, they do not drive). The SPOT points
    //  are used only if the drive-to-spot version of the aim is started.
    // =========================================================

    /** Right bumper: aim the back of the robot at the right goal (the spot is the old drive-to point). */
    public static final double SPOT_RIGHT_X = 118.0;
    public static final double SPOT_RIGHT_Y = 55.5;
    public static final double GOAL_RIGHT_X = 92.0;
    public static final double GOAL_RIGHT_Y = 58.0;

    /** Left bumper: the mirror image of the right side across the field center (x = 72). */
    public static final double SPOT_LEFT_X = 23.0;
    public static final double SPOT_LEFT_Y = 55.5;
    public static final double GOAL_LEFT_X = 49.0;
    public static final double GOAL_LEFT_Y = 59.0;

    /** How the Control Hub is mounted on the robot (for the hub IMU): logo faces left, USB ports point up. */
    public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    /** Teleop heading hold (Gamepad 1 bumpers): field headings to turn to and hold while held. */
    public static final double HEADING_HOLD_RIGHT_DEG = 90.0;
    public static final double HEADING_HOLD_LEFT_DEG = -90.0;

    /** Top power while driving to the spot. */
    public static final double SPOT_APPROACH_MAX_POWER = 0.7;

    /** The robot has arrived when it is this close to the spot (inches). */
    public static final double SPOT_ARRIVE_TOL_IN = 1.0;

    /** Aim is good within this many degrees; it re-aims only after drifting past RELOCK. */
    public static final double AIM_TOLERANCE_DEG = 1.5;
    public static final double AIM_RELOCK_DEG = 3.0;

    /** The aim must stay good this long before the controller buzzes "locked". */
    public static final long AIM_SETTLE_MS = 150;

    // =========================================================
    //  FIELD DRIVE  (Biobuzz Field Drive OpMode)
    //  Starting values - tune on the robot. These are not final so the Field Drive Tuning OpMode
    //  can change them live; copy the values you settle on back into this file.
    // =========================================================

    /** A stick must be pushed past this (0 to 1) to count as a direction. */
    public static double FD_STICK_DEADBAND = 0.5;

    /**
     * While a stick is returning toward center and is below this deflection, its angle is ignored,
     * so letting go never swings the robot toward a wandering angle. 0 turns this off.
     */
    public static double FD_STICK_FREEZE = 0.75;

    /** Power with a stick held and the trigger untouched. The trigger adds the rest, up to 100%. */
    public static double FD_THROTTLE_FLOOR = 0.20;

    /** No forward power when the heading error is beyond CUTOFF; full power within FULL (degrees). */
    public static double FD_ALIGN_CUTOFF_DEG = 30.0;
    public static double FD_ALIGN_FULL_DEG = 5.0;

    /**
     * Turning: power per degree of heading error, and damping per degree/second of turn rate.
     * Turned down from the first values, which made the turning jerky.
     */
    public static double FD_TURN_KP = 0.018;
    public static double FD_TURN_KD = 0.0015;

    /** Turning power limits. The minimum keeps it from crawling the last few degrees. */
    public static double FD_TURN_MAX_POWER = 0.6;
    public static double FD_TURN_MIN_POWER = 0.08;

    /**
     * Most turn power allowed while driving at full speed. The turn limit blends from
     * FD_TURN_MAX_POWER (standing still) down to this as the drive power rises to 100%.
     */
    public static double FD_TURN_MAX_AT_SPEED = 0.35;

    /**
     * Limits how fast turning power can build up, in power per second (0 = no limit). It only slows
     * the start of a turn; braking is never delayed. Raise it for snappier turns.
     */
    public static double FD_TURN_RAMP_RATE = 6.0;

    /** The robot is on its heading within this many degrees, and stops turning. */
    public static double FD_TURN_TOLERANCE_DEG = 2.5;

    /**
     * Heading + Drive version (right stick = heading, left stick = speed): the left stick's top power
     * with the trigger untouched. The left trigger raises that top power toward 100%.
     */
    public static double FD_ARCADE_MAX_POWER = 0.6;

    /** Heading + Drive version: the left stick must move past this (0 to 1) to drive. */
    public static double FD_ARCADE_DEADBAND = 0.05;

    /** Forward/back power ramp, in power per second. Deceleration is gentler to stop the rocking. */
    public static double FD_ACCEL_RATE = 3.0;
    public static double FD_DECEL_RATE = 2.0;

    /**
     * Limits how fast each side's wheel power can change, in power per second (0 = no limit).
     * It stops an inside wheel from reversing in a snap during a sharp turn.
     */
    public static double FD_WHEEL_RAMP_RATE = 8.0;

    /** Smoothing for the turn rate worked out from the heading (0 to 1; higher = less smoothing). */
    public static double FD_RATE_FILTER = 0.4;

    // =========================================================
    //  HELPERS
    // =========================================================

    /** Moves current toward target by at most maxStep. */
    public static double slew(double current, double target, double maxStep) {
        return current + Math.max(-maxStep, Math.min(maxStep, target - current));
    }

    // =========================================================
    //  DRIVE RAMP
    // =========================================================

    /** Seconds for drive/turn power to ramp across its full range (0 to 1). Raise to soften starts and stops. */
    public static final double DRIVE_RAMP_TIME_SEC = 0.3;

    /** Seconds for turn power to ramp across its full range. Kept short so steering stays crisp. */
    public static final double TURN_RAMP_TIME_SEC = 0.05;

    // =========================================================
    //  GATE SERVO (ON/OFF)
    // =========================================================

    /** Servo position with the gate closed (balls held back). */
    public static final double GATE_CLOSED_POSITION = 1.0;

    /** Servo position with the gate open (balls released). Set this to match the robot. */
    public static final double GATE_OPEN_POSITION = 0.0;

    /** The right trigger must be pulled past this (0 to 1) to open the gate. */
    public static final double GATE_TRIGGER_THRESHOLD = 0.5;
}
