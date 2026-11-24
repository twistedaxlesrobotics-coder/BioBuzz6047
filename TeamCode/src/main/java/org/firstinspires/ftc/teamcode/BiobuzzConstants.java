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

    /**
     * Right bumper: aim the back of the robot at the right goal (the spot is the old drive-to point).
     * The goal is measured from the field center: 19 in to the right, 12.25 in toward the drivers.
     */
    public static final double SPOT_RIGHT_X = 46.0;
    public static final double SPOT_RIGHT_Y = -16.5;
    public static final double GOAL_RIGHT_X = 19.0;
    public static final double GOAL_RIGHT_Y = -12.25;

    /** Left bumper: the mirror image of the right goal across the field center (x = 0). */
    public static final double SPOT_LEFT_X = -49.0;
    public static final double SPOT_LEFT_Y = -16.5;
    public static final double GOAL_LEFT_X = -19.0;
    public static final double GOAL_LEFT_Y = -12.25;

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
    //  CAMERA (APRILTAG RELOCALIZATION)
    //  Each time the robot shoots, the camera looks for a goal tag and corrects the pose X/Y.
    //  Heading is NOT corrected: the Pinpoint heading is trusted, and one tag gives a noisy heading.
    // =========================================================

    /** Turn the camera correction on or off in teleop (Field Drive) and in the autos. */
    public static final boolean CAM_ENABLED_TELEOP = true;
    public static final boolean CAM_ENABLED_AUTO = true;

    /** Name of the webcam in the robot configuration, and the mode it runs in. */
    public static final String CAM_NAME = "Webcam 1";
    public static final int CAM_WIDTH = 640;
    public static final int CAM_HEIGHT = 480;

    /** Lens calibration for 640x480 from last season (fx, fy, cx, cy). Check it with Camera Check. */
    public static final double CAM_FX = 539.0239404;
    public static final double CAM_FY = 539.0239404;
    public static final double CAM_CX = 316.450283269;
    public static final double CAM_CY = 236.364794005;

    /** Manual exposure (ms) and gain, from last season. Tune with a camera tuner if the tags are missed. */
    public static final int CAM_EXPOSURE_MS = 1;
    public static final int CAM_GAIN = 85;

    /**
     * Where the camera lens is on the robot, from the robot's center of rotation (inches).
     * X: right is positive (the camera is on the LEFT, so negative). Y: forward is positive
     * (the camera is BEHIND center, so negative). The camera points straight back, tilted up.
     */
    public static final double CAM_MOUNT_X_IN = -3.5;
    public static final double CAM_MOUNT_Y_IN = -0.75;
    public static final double CAM_MOUNT_HEIGHT_IN = 16.5;
    public static final double CAM_TILT_UP_DEG = 45.0;

    /** Which alliance we are. Only red is set up so far. */
    public static final boolean ALLIANCE_IS_RED = true;

    /**
     * The four AprilTags on the underside of each hive, for shooting to the left and to the right
     * (red alliance). The left bumper uses the left tags and the right bumper the right tags.
     * Where each tag IS on the field is not typed in: Camera Check learns it from a known spot with
     * the hive up, and saves it on the robot. Until a tag is learned it is not used.
     */
    public static final int[] CAM_RED_LEFT_TAGS = {30, 31, 32, 33};
    public static final int[] CAM_RED_RIGHT_TAGS = {34, 35, 36, 37};

    /**
     * PLACEHOLDERS for blue: fill in the tag numbers when blue is sorted out, then set
     * ALLIANCE_IS_RED to false. Empty means no tags, so the camera does nothing for blue.
     */
    public static final int[] CAM_BLUE_LEFT_TAGS = {};
    public static final int[] CAM_BLUE_RIGHT_TAGS = {};

    /**
     * Where the robot is placed at the start of each auto (field center origin, inches and degrees).
     * The autos and Camera Check both use these, so they stay the same place.
     */
    public static final double START_RIGHT_X = 59.5;
    public static final double START_RIGHT_Y = -14.5;
    public static final double START_RIGHT_HEADING = 90.0;
    public static final double START_LEFT_X = -59.5;
    public static final double START_LEFT_Y = -14.5;
    public static final double START_LEFT_HEADING = -90.0;

    /**
     * A third spot for Camera Check to learn tags from: any spot you can place the robot at exactly.
     * Leave X and Y as NaN to leave it unused. Heading is in the field frame (0 faces +Y, clockwise).
     */
    public static final double CAM_LEARN_X = Double.NaN;
    public static final double CAM_LEARN_Y = Double.NaN;
    public static final double CAM_LEARN_HEADING = 0.0;

    /** After the gate opens, the camera has this long to get a fix (teleop), or this long before a shot (auto). */
    public static final double CAM_WINDOW_S = 1.0;
    public static final double CAM_AUTO_TIMEOUT_S = 0.8;

    /** Frames averaged for one fix, and the least time between frames used (so a frame is not counted twice). */
    public static final int CAM_FRAMES_NEEDED = 4;
    public static final long CAM_SAMPLE_INTERVAL_MS = 35;

    /** Ignore tags farther than this from the camera (inches). */
    public static final double CAM_MAX_RANGE_IN = 110.0;

    /** The frames must agree to within this (inches) or the fix is thrown out. */
    public static final double CAM_MAX_SPREAD_IN = 2.0;

    /** A correction bigger than this (inches) is rejected as a bad reading. */
    public static final double CAM_MAX_JUMP_IN = 12.0;

    /** A fix from only ONE tag is rejected if it moves the robot more than this (inches). */
    public static final double CAM_SINGLE_TAG_MAX_JUMP_IN = 6.0;

    /**
     * Tags seen together in one frame must agree on where the robot is within this (inches), or the
     * frame is skipped. A hive that is not in its normal up position makes them disagree.
     */
    public static final double CAM_MAX_TAG_DISAGREE_IN = 3.0;

    /** A correction smaller than this (inches) is not worth applying. */
    public static final double CAM_MIN_CORRECTION_IN = 0.5;

    /** How much of the correction to apply: 1.0 = jump to the camera position, lower = move partway. */
    public static final double CAM_BLEND = 1.0;

    /** The robot counts as still when it moves slower than this (inches/second) and turns slower than this (degrees/second). */
    public static final double CAM_STILL_SPEED_IN_S = 4.0;
    public static final double CAM_STILL_TURN_DPS = 20.0;

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
