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
    //  GATE SERVO (ON/OFF)
    // =========================================================

    /** Servo position with the gate closed (balls held back). */
    public static final double GATE_CLOSED_POSITION = 1.0;

    /** Servo position with the gate open (balls released). Set this to match the robot. */
    public static final double GATE_OPEN_POSITION = 0.0;

    /** The right trigger must be pulled past this (0 to 1) to open the gate. */
    public static final double GATE_TRIGGER_THRESHOLD = 0.5;
}
