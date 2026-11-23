package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Tunes the field drive live. Gamepad 1 drives exactly like Biobuzz Field Drive: point and go (right
 * stick = front, left stick = back, left trigger = accelerator), or heading + drive (right stick =
 * heading, left stick = forward/back, left trigger = boost); D-pad right on Gamepad 1 switches.
 * Gamepad 2 changes the settings:
 * - D-pad up / down: pick a setting (marked with &gt;)
 * - D-pad right / left: raise / lower the picked setting by its step
 * - X: reset the pose to the known reset position (and hold the new heading)
 *
 * The screen lists every setting as NAME = value. When it drives the way you want, copy the values
 * into the FD_ constants in BiobuzzConstants. Changes last only until the app restarts.
 * The shooter, intake and gate are not used here.
 */
@TeleOp(name = "Field Drive Tuning", group = "Tuning")
public class FieldDriveTuningOpMode extends LinearOpMode {

    private static final String[] NAMES = {
            "FD_STICK_DEADBAND",
            "FD_STICK_FREEZE",
            "FD_THROTTLE_FLOOR",
            "FD_ALIGN_CUTOFF_DEG",
            "FD_ALIGN_FULL_DEG",
            "FD_TURN_KP",
            "FD_TURN_KD",
            "FD_TURN_MAX_POWER",
            "FD_TURN_MIN_POWER",
            "FD_TURN_TOLERANCE_DEG",
            "FD_TURN_RAMP_RATE",
            "FD_ACCEL_RATE",
            "FD_DECEL_RATE",
            "FD_RATE_FILTER",
            "FD_ARCADE_MAX_POWER"
    };

    private static final double[] STEPS = {
            0.05,    // deadband
            0.05,    // freeze
            0.05,    // throttle floor
            5.0,     // align cutoff
            1.0,     // align full
            0.002,   // turn kP
            0.0002,  // turn kD
            0.05,    // turn max power
            0.01,    // turn min power
            0.5,     // turn tolerance
            1.0,     // turn ramp rate
            0.25,    // accel rate
            0.25,    // decel rate
            0.05,    // rate filter
            0.05     // heading + drive top power
    };

    BiobuzzHardware robot = new BiobuzzHardware();
    BiobuzzAutoDrive pose = null;
    BiobuzzFieldDrive fieldDrive = new BiobuzzFieldDrive();
    ElapsedTime driveTimer = new ElapsedTime();

    int selected = 0;
    boolean prevUp, prevDown, prevLeft, prevRight, prevX;

    // Drive version (Gamepad 1 D-pad right): false = point and go, true = heading + drive
    boolean headingArcade = false;
    boolean prevDpadRight1 = false;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        try {
            pose = new BiobuzzAutoDrive(this, robot);
            pose.initWithoutCalibration();
            pose.setStartPose(PoseStore.getX(), PoseStore.getY(), PoseStore.getHeading());
            fieldDrive.holdHeadingNow(pose.getHeading());
        } catch (Exception e) {
            pose = null;
        }

        telemetry.addData("Status", pose == null ? "Pinpoint not found - drivetrain off" : "Initialized");
        telemetry.update();

        waitForStart();
        driveTimer.reset();

        while (opModeIsActive()) {

            // --- Pick and change a setting (Gamepad 2) ---

            if (gamepad2.dpad_up && !prevUp) {
                selected = (selected + NAMES.length - 1) % NAMES.length;
            }
            if (gamepad2.dpad_down && !prevDown) {
                selected = (selected + 1) % NAMES.length;
            }
            if (gamepad2.dpad_right && !prevRight) {
                change(selected, +1);
            }
            if (gamepad2.dpad_left && !prevLeft) {
                change(selected, -1);
            }
            prevUp = gamepad2.dpad_up;
            prevDown = gamepad2.dpad_down;
            prevRight = gamepad2.dpad_right;
            prevLeft = gamepad2.dpad_left;

            // --- Drive (Gamepad 1) ---

            double dt = driveTimer.seconds();
            driveTimer.reset();

            boolean poseReset = gamepad2.x && !prevX;
            prevX = gamepad2.x;

            if (pose == null) {
                robot.setDrivePower(0, 0, 0, 0);
            } else {
                if (poseReset) {
                    pose.setStartPose(PoseStore.RESET_X, PoseStore.RESET_Y, PoseStore.RESET_HEADING);
                }
                pose.update();
                PoseStore.save(pose.getX(), pose.getY(), pose.getHeading());
                if (poseReset) {
                    fieldDrive.holdHeadingNow(pose.getHeading());
                }

                if (gamepad1.dpad_right && !prevDpadRight1) {
                    headingArcade = !headingArcade;
                    fieldDrive.holdHeadingNow(pose.getHeading());
                }
                prevDpadRight1 = gamepad1.dpad_right;

                if (headingArcade) {
                    fieldDrive.updateHeadingArcade(
                            gamepad1.right_stick_x, gamepad1.right_stick_y,
                            gamepad1.left_stick_y,
                            gamepad1.left_trigger,
                            pose.getHeading(),
                            dt
                    );
                } else {
                    fieldDrive.update(
                            gamepad1.left_stick_x, gamepad1.left_stick_y,
                            gamepad1.right_stick_x, gamepad1.right_stick_y,
                            gamepad1.left_trigger,
                            pose.getHeading(),
                            dt
                    );
                }
                robot.setDrivePower(
                        fieldDrive.leftPower,
                        fieldDrive.leftPower,
                        fieldDrive.rightPower,
                        fieldDrive.rightPower
                );
            }

            // --- Telemetry ---

            telemetry.addLine("Gamepad 2: D-pad up/down pick, left/right change, X reset pose");
            for (int i = 0; i < NAMES.length; i++) {
                telemetry.addData((i == selected ? "> " : "  ") + NAMES[i], "%.4f", get(i));
            }

            if (pose != null) {
                pose.addPoseTelemetry();
                telemetry.addData("Drive Mode", headingArcade ? "HEADING + DRIVE" : "POINT AND GO");
                telemetry.addData("Active Stick", fieldDrive.activeStick);
                telemetry.addData("Target Heading", "%.1f%s", fieldDrive.targetHeading,
                        fieldDrive.targetFrozen ? " (frozen)" : "");
                telemetry.addData("Heading Error", "%.1f", fieldDrive.headingError);
                telemetry.addData("Turn Rate (deg/s)", "%.0f", fieldDrive.turnRateDps);
                telemetry.addData("Throttle", "%.2f (align %.2f)", fieldDrive.throttle,
                        fieldDrive.alignScale);
                telemetry.addData("Drive Power", "%.2f", fieldDrive.drivePower);
                telemetry.addData("Left / Right", "%.2f / %.2f",
                        fieldDrive.leftPower, fieldDrive.rightPower);
            } else {
                telemetry.addData("ERROR", "Pinpoint not found - drivetrain off");
            }

            telemetry.update();
        }
    }

    /** Moves a setting one step up (+1) or down (-1), never below zero. */
    private void change(int index, int direction) {
        double value = get(index) + direction * STEPS[index];
        value = Math.max(0.0, Math.round(value * 1e6) / 1e6);
        set(index, value);
    }

    private double get(int index) {
        switch (index) {
            case 0:  return BiobuzzConstants.FD_STICK_DEADBAND;
            case 1:  return BiobuzzConstants.FD_STICK_FREEZE;
            case 2:  return BiobuzzConstants.FD_THROTTLE_FLOOR;
            case 3:  return BiobuzzConstants.FD_ALIGN_CUTOFF_DEG;
            case 4:  return BiobuzzConstants.FD_ALIGN_FULL_DEG;
            case 5:  return BiobuzzConstants.FD_TURN_KP;
            case 6:  return BiobuzzConstants.FD_TURN_KD;
            case 7:  return BiobuzzConstants.FD_TURN_MAX_POWER;
            case 8:  return BiobuzzConstants.FD_TURN_MIN_POWER;
            case 9:  return BiobuzzConstants.FD_TURN_TOLERANCE_DEG;
            case 10: return BiobuzzConstants.FD_TURN_RAMP_RATE;
            case 11: return BiobuzzConstants.FD_ACCEL_RATE;
            case 12: return BiobuzzConstants.FD_DECEL_RATE;
            case 13: return BiobuzzConstants.FD_RATE_FILTER;
            default: return BiobuzzConstants.FD_ARCADE_MAX_POWER;
        }
    }

    private void set(int index, double value) {
        switch (index) {
            case 0:  BiobuzzConstants.FD_STICK_DEADBAND = value; break;
            case 1:  BiobuzzConstants.FD_STICK_FREEZE = value; break;
            case 2:  BiobuzzConstants.FD_THROTTLE_FLOOR = value; break;
            case 3:  BiobuzzConstants.FD_ALIGN_CUTOFF_DEG = value; break;
            case 4:  BiobuzzConstants.FD_ALIGN_FULL_DEG = value; break;
            case 5:  BiobuzzConstants.FD_TURN_KP = value; break;
            case 6:  BiobuzzConstants.FD_TURN_KD = value; break;
            case 7:  BiobuzzConstants.FD_TURN_MAX_POWER = value; break;
            case 8:  BiobuzzConstants.FD_TURN_MIN_POWER = value; break;
            case 9:  BiobuzzConstants.FD_TURN_TOLERANCE_DEG = value; break;
            case 10: BiobuzzConstants.FD_TURN_RAMP_RATE = value; break;
            case 11: BiobuzzConstants.FD_ACCEL_RATE = value; break;
            case 12: BiobuzzConstants.FD_DECEL_RATE = value; break;
            case 13: BiobuzzConstants.FD_RATE_FILTER = value; break;
            default: BiobuzzConstants.FD_ARCADE_MAX_POWER = value; break;
        }
    }
}
