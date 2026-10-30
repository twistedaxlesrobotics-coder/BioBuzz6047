package org.firstinspires.ftc.teamcode;

/**
 * BiobuzzConstants
 * Stores constants for shooter presets and tolerances, ported from last year's Decode logic.
 */
public class BiobuzzConstants {

    // =========================================================
    //  SHOOTER (SPEEDS + TOLERANCES)
    // =========================================================

    public enum ShooterPreset { STOP, CLOSE, }

    // Preset RPM values (mimicking last year's top motor speeds)
    public static final double RPM_CLOSE = 2500.0;

    /**
     * Look up the target RPM for a specific preset.
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
