package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Field-centric teleop with two drive versions, switched with Gamepad 1 D-pad right. See
 * BiobuzzFieldDrive for the details.
 *
 * Needs the Pinpoint. If it is not found, the drivetrain stays off and a message is shown; use
 * Biobuzz Arcade Drive instead.
 *
 * Controls (Gamepad 1):
 * - POINT AND GO (the default): right stick = front direction, left stick = back direction, left
 *   trigger = accelerator.
 * - HEADING + DRIVE: right stick = turn to a heading (no movement), left stick = forward/back
 *   (top power 60%), left trigger = boost toward 100%.
 * - Both: right trigger = gate, bumpers = aim at the goal (D-pad left switches them to the fixed
 *   headings), D-pad up/down = start/stop the shooter, D-pad right = switch drive version.
 * Gamepad 2: bumpers = intake, D-pad up = reset the pose to the known reset position.
 */
@TeleOp(name = "Biobuzz Field Drive", group = "Linear Opmode")
public class BiobuzzFieldDriveTeleOp extends LinearOpMode {

    BiobuzzHardware robot = new BiobuzzHardware();

    // Shooter state
    BiobuzzConstants.ShooterPreset targetPreset = BiobuzzConstants.ShooterPreset.STOP;

    // Field position from the Pinpoint (null if the sensor is not available)
    BiobuzzAutoDrive pose = null;
    boolean prevPoseReset = false;

    // The field drive and its timing
    BiobuzzFieldDrive fieldDrive = new BiobuzzFieldDrive();
    ElapsedTime driveTimer = new ElapsedTime();

    // AprilTag camera: each time the gate opens it looks for a goal tag and corrects the position
    // (null if the camera is turned off or not found)
    BiobuzzVision vision = null;
    BiobuzzRelocalizer relocalizer = null;
    boolean prevGateOpen = false;

    // Bumper assist: 0 = off, 1 = left, 2 = right. Aim at the goal by default, or the fixed headings.
    BiobuzzSpotAim spotAim = null;
    BiobuzzHeadingHold headingHold = new BiobuzzHeadingHold();
    boolean xyAimMode = false;
    boolean prevDpadLeft = false;

    // Drive version (Gamepad 1 D-pad right): false = point and go, true = heading + drive
    boolean headingArcade = false;
    boolean prevDpadRight = false;
    int assistSelection = 0;
    boolean prevAimLocked = false;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Pick up the pose the auto ended at, or the reset pose if there isn't one
        try {
            pose = new BiobuzzAutoDrive(this, robot);
            pose.initWithoutCalibration();
            pose.setStartPose(
                    PoseStore.getX(),
                    PoseStore.getY(),
                    PoseStore.getHeading()
            );
            spotAim = new BiobuzzSpotAim(pose);
            xyAimMode = true;   // bumpers aim at the goal points by default
            fieldDrive.holdHeadingNow(pose.getHeading());
        } catch (Exception e) {
            pose = null;
            spotAim = null;
        }

        // The camera is optional: if it is missing, the robot still drives
        if (pose != null && BiobuzzConstants.CAM_ENABLED_TELEOP) {
            try {
                vision = new BiobuzzVision(hardwareMap);
                relocalizer = new BiobuzzRelocalizer(vision, pose);
            } catch (Exception e) {
                vision = null;
                relocalizer = null;
            }
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Drive Mode", "Field Drive");
        if (pose == null) {
            telemetry.addData("ERROR", "Pinpoint not found - drivetrain off. Use Biobuzz Arcade Drive.");
        }
        telemetry.update();

        waitForStart();
        driveTimer.reset();

        while (opModeIsActive()) {

            // --- Field Pose (Pinpoint) ---

            // Failsafe: Gamepad 2 D-pad up resets the pose and heading to the known reset position
            boolean poseReset = gamepad2.dpad_up && !prevPoseReset;
            prevPoseReset = gamepad2.dpad_up;

            if (pose != null) {
                if (poseReset) {
                    pose.setStartPose(
                            PoseStore.RESET_X,
                            PoseStore.RESET_Y,
                            PoseStore.RESET_HEADING
                    );
                }

                pose.update();
                PoseStore.save(pose.getX(), pose.getY(), pose.getHeading());

                // The heading just jumped, so hold the new one instead of turning back to the old one
                if (poseReset) {
                    fieldDrive.holdHeadingNow(pose.getHeading());
                }

                // Camera: each time the gate opens, look for a goal tag and correct the position.
                // Only X and Y change, so the heading hold is not disturbed.
                boolean gateOpen = gamepad1.right_trigger > BiobuzzConstants.GATE_TRIGGER_THRESHOLD;
                if (relocalizer != null) {
                    if (gateOpen && !prevGateOpen) {
                        relocalizer.request(BiobuzzConstants.CAM_WINDOW_S);
                    }
                    relocalizer.update();
                }
                prevGateOpen = gateOpen;
            }

            double dt = driveTimer.seconds();
            driveTimer.reset();


            // --- Drivetrain ---

            if (pose == null) {
                robot.setDrivePower(0, 0, 0, 0);
            } else {

                // Drive version toggle (Gamepad 1 D-pad right): point and go, or heading + drive.
                // The robot holds the heading it has, with fresh ramps, so nothing lurches.
                if (gamepad1.dpad_right && !prevDpadRight) {
                    headingArcade = !headingArcade;
                    fieldDrive.holdHeadingNow(pose.getHeading());
                    gamepad1.rumble(0.5, 0.5, 250);
                }
                prevDpadRight = gamepad1.dpad_right;

                // Bumper mode toggle (Gamepad 1 D-pad left): aim at the goal, or the fixed headings
                if (gamepad1.dpad_left && !prevDpadLeft) {
                    xyAimMode = !xyAimMode;
                    // Drop any assist in progress so the new mode starts clean
                    spotAim.stop();
                    headingHold.stop();
                    assistSelection = 0;
                    prevAimLocked = false;
                    gamepad1.rumble(0.3, 0.3, 150);
                }
                prevDpadLeft = gamepad1.dpad_left;

                // Bumper assist: hold a bumper to aim. The sticks and trigger are ignored while it
                // is held; release to drive.
                boolean wantLeft = gamepad1.left_bumper && !gamepad1.right_bumper;
                boolean wantRight = gamepad1.right_bumper && !gamepad1.left_bumper;
                int wanted = wantLeft ? 1 : (wantRight ? 2 : 0);

                if (wanted != assistSelection) {
                    assistSelection = wanted;
                    if (xyAimMode) {
                        if (wanted == 1) {
                            spotAim.startAimOnly(
                                    BiobuzzConstants.GOAL_LEFT_X, BiobuzzConstants.GOAL_LEFT_Y);
                        } else if (wanted == 2) {
                            spotAim.startAimOnly(
                                    BiobuzzConstants.GOAL_RIGHT_X, BiobuzzConstants.GOAL_RIGHT_Y);
                        } else {
                            spotAim.stop();
                        }
                    } else if (wanted == 1) {
                        headingHold.start(BiobuzzConstants.HEADING_HOLD_LEFT_DEG);
                    } else if (wanted == 2) {
                        headingHold.start(BiobuzzConstants.HEADING_HOLD_RIGHT_DEG);
                    } else {
                        headingHold.stop();
                    }
                }

                boolean assistActive = false;
                double assistLeft = 0.0;
                double assistRight = 0.0;
                boolean locked;
                if (xyAimMode) {
                    if (wanted != 0) {
                        spotAim.update();
                        assistLeft = spotAim.leftPower;
                        assistRight = spotAim.rightPower;
                        assistActive = true;
                    }
                    locked = spotAim.isLocked();
                } else {
                    if (wanted != 0) {
                        headingHold.update(pose.getHeading());
                        assistLeft = headingHold.leftPower;
                        assistRight = headingHold.rightPower;
                        assistActive = true;
                    }
                    locked = headingHold.isLocked();
                }

                // Buzz once when the robot is aimed and settled
                if (locked && !prevAimLocked) {
                    gamepad1.rumble(0.6, 0.6, 300);
                }
                prevAimLocked = locked;

                if (assistActive) {
                    robot.setDrivePower(assistLeft, assistLeft, assistRight, assistRight);
                    // Keep the field drive on the heading the assist leaves the robot at, with a
                    // fresh ramp, so control returns smoothly when the bumper is released
                    fieldDrive.holdHeadingNow(pose.getHeading());
                } else {
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
            }


            // --- Intake Control (Gamepad 2) ---

            if (gamepad2.left_bumper) {
                robot.intakeMotor.setPower(1.0);
            } else if (gamepad2.right_bumper) {
                robot.intakeMotor.setPower(-1.0);
            } else {
                robot.intakeMotor.setPower(0.0);
            }


            // --- Gate Servo Control (Gamepad 1 Right Trigger) ---

            robot.setGate(gamepad1.right_trigger > BiobuzzConstants.GATE_TRIGGER_THRESHOLD);


            // --- Shooter Control ---

            // Start the shooter (CLOSE) on Gamepad 1 D-pad up, stop it on D-pad down
            if (gamepad1.dpad_up) {
                targetPreset = BiobuzzConstants.ShooterPreset.CLOSE;
            } else if (gamepad1.dpad_down) {
                targetPreset = BiobuzzConstants.ShooterPreset.STOP;
            }

            double targetRpm = BiobuzzConstants.getTargetRpm(targetPreset);
            robot.setShooterWheelRpm(targetRpm);

            // Apply voltage compensation to shooter PIDF
            robot.applyShooterPIDFWithVoltageComp();


            // --- Telemetry ---

            telemetry.addData("Drive Mode",
                    headingArcade ? "HEADING + DRIVE" : "POINT AND GO");

            if (pose != null) {
                pose.addPoseTelemetry();
                if (relocalizer != null) {
                    relocalizer.addTelemetry(telemetry);
                }
                telemetry.addData("Bumper Mode",
                        xyAimMode ? "AIM AT GOAL (field point)" : "FIXED HEADING");
                if (xyAimMode) {
                    telemetry.addData("Aim", spotAim.getStatus());
                } else {
                    telemetry.addData("Heading Hold", headingHold.getStatus());
                }
                telemetry.addData("Active Stick", fieldDrive.activeStick);
                telemetry.addData("Target Heading", "%.1f%s", fieldDrive.targetHeading,
                        fieldDrive.targetFrozen ? " (frozen)" : "");
                telemetry.addData("Heading Error", "%.1f", fieldDrive.headingError);
                telemetry.addData("Throttle", "%.2f (align %.2f)", fieldDrive.throttle,
                        fieldDrive.alignScale);
                telemetry.addData("Drive Power", "%.2f", fieldDrive.drivePower);
                telemetry.addData("Left / Right", "%.2f / %.2f",
                        fieldDrive.leftPower, fieldDrive.rightPower);
            } else {
                telemetry.addData("ERROR", "Pinpoint not found - drivetrain off");
            }

            telemetry.addData("Intake Status",
                    robot.intakeMotor.getPower() != 0
                            ? (robot.intakeMotor.getPower() > 0 ? "FORWARD" : "REVERSE")
                            : "STOPPED");

            telemetry.addData("Gate",
                    gamepad1.right_trigger > BiobuzzConstants.GATE_TRIGGER_THRESHOLD
                            ? "OPEN" : "CLOSED");

            telemetry.addData("Shooter Preset", targetPreset);
            telemetry.addData("Shooter RPM", "Target: %.0f, Actual: %.0f",
                    targetRpm, robot.getShooterWheelRpm());
            telemetry.addData("Shooter Ready",
                    robot.isShooterAtSpeed(targetRpm, BiobuzzConstants.SHOOTER_RPM_TOLERANCE)
                            ? "READY" : "WAITING");

            telemetry.update();
        }

        if (vision != null) {
            vision.close();
        }
    }
}
