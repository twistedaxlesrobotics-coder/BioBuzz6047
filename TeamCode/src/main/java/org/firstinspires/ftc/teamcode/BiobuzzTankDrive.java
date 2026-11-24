
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Biobuzz Arcade Drive", group = "Linear Opmode")
public class BiobuzzTankDrive extends LinearOpMode {

    // Declare robot hardware
    BiobuzzHardware robot = new BiobuzzHardware();

    // Shooter state
    BiobuzzConstants.ShooterPreset targetPreset =
            BiobuzzConstants.ShooterPreset.STOP;

    // Drive state
    boolean isReversed = false;

    // Button state for detecting D-pad presses
    boolean prevDpadRight = false;

    // Drive ramp state (limits how fast drive/turn power can change)
    double rampedDrive = 0.0;
    double rampedTurn = 0.0;
    ElapsedTime rampTimer = new ElapsedTime();

    // Field position from the Pinpoint (null if the sensor is not available)
    BiobuzzAutoDrive pose = null;
    boolean prevPoseReset = false;

    // Heading hold (Gamepad 1 bumpers): 0 = off, 1 = left, 2 = right
    BiobuzzHeadingHold headingHold = null;
    int assistSelection = 0;
    boolean prevAimLocked = false;

    // Aim-at-goal mode for the bumpers (Gamepad 1 D-pad left toggles it). Off = the fixed headings.
    // Needs the Pinpoint pose, so it is null (and the fixed headings are used) if the sensor is
    // not available.
    BiobuzzSpotAim spotAim = null;
    boolean xyAimMode = false;
    boolean prevDpadLeft = false;

    // AprilTag camera: each time the gate opens it looks for a goal tag and corrects the position
    // (null if the camera is turned off or not found)
    BiobuzzVision vision = null;
    BiobuzzRelocalizer relocalizer = null;
    boolean prevGateOpen = false;

    // Field heading for the hold comes from the hub IMU: heading = -yaw + offset (null if no IMU)
    IMU imu = null;
    double imuHeadingOffset = 0.0;

    // Straight-drive gyro assist: the heading being held while driving straight
    boolean straightHolding = false;
    double straightHeading = 0.0;

    @Override
    public void runOpMode() {

        // Initialize robot hardware
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

        // Start the hub IMU at the heading the auto ended with (or the reset heading)
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    BiobuzzConstants.HUB_LOGO_DIRECTION,
                    BiobuzzConstants.HUB_USB_DIRECTION)));
            imu.resetYaw();
            imuHeadingOffset = PoseStore.getHeading();
            headingHold = new BiobuzzHeadingHold();
        } catch (Exception e) {
            imu = null;
            headingHold = null;
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Drive Mode", "Arcade");
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();
        rampTimer.reset();

        // Main robot control loop
        while (opModeIsActive()) {

            // --- Toggle Front/Back Orientation (D-pad Right) ---

            if (gamepad1.dpad_right && !prevDpadRight) {
                isReversed = !isReversed;
            }

            prevDpadRight = gamepad1.dpad_right;


            // --- Field Pose (Pinpoint) ---

            // Failsafe: Gamepad 2 D-pad up resets the pose and heading to the known reset position
            boolean poseReset = gamepad2.dpad_up && !prevPoseReset;
            prevPoseReset = gamepad2.dpad_up;

            if (poseReset && imu != null) {
                imuHeadingOffset = PoseStore.RESET_HEADING + getImuYaw();
            }

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

                // Camera: each time the gate opens, look for a goal tag and correct the position.
                // Only X and Y change, so the IMU heading hold is not disturbed.
                boolean gateOpen = gamepad1.right_trigger > BiobuzzConstants.GATE_TRIGGER_THRESHOLD;
                if (relocalizer != null) {
                    if (gateOpen && !prevGateOpen) {
                        // The bumper being held says which hive we are shooting at
                        relocalizer.request(BiobuzzConstants.CAM_WINDOW_S,
                                BiobuzzRelocalizer.sideFromBumper(assistSelection));
                    }
                    relocalizer.update();
                }
                prevGateOpen = gateOpen;
            }


            // --- Bumper Mode Toggle (Gamepad 1 D-pad left) ---
            // Switches the bumpers between aiming at a field point (the default) and the fixed
            // headings. Ignored if the Pinpoint pose is not available.

            if (gamepad1.dpad_left && !prevDpadLeft && spotAim != null) {
                xyAimMode = !xyAimMode;
                // Drop any assist in progress so the new mode starts clean
                spotAim.stop();
                if (headingHold != null) {
                    headingHold.stop();
                }
                assistSelection = 0;
                prevAimLocked = false;
                gamepad1.rumble(0.3, 0.3, 150);
            }
            prevDpadLeft = gamepad1.dpad_left;


            // --- Bumper Assist (Gamepad 1 bumpers) ---
            // Hold the right or left bumper. Aim mode: pivot in place so the back of the robot (the
            // shooter) faces that side's goal point, and keep it aimed from the live pose. It never
            // drives. Fixed heading mode: turn to that side's heading and hold it. The sticks are
            // ignored while it is held; release to drive.

            boolean assistActive = false;
            double assistLeft = 0.0;
            double assistRight = 0.0;
            boolean useSpotAim = xyAimMode && spotAim != null;
            if (useSpotAim || headingHold != null) {
                boolean wantLeft = gamepad1.left_bumper && !gamepad1.right_bumper;
                boolean wantRight = gamepad1.right_bumper && !gamepad1.left_bumper;
                int wanted = wantLeft ? 1 : (wantRight ? 2 : 0);

                if (wanted != assistSelection) {
                    assistSelection = wanted;
                    if (useSpotAim) {
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

                boolean locked;
                if (useSpotAim) {
                    if (wanted != 0) {
                        spotAim.update();
                        assistLeft = spotAim.leftPower;
                        assistRight = spotAim.rightPower;
                        assistActive = true;
                    }
                    locked = spotAim.isLocked();
                } else {
                    if (wanted != 0) {
                        headingHold.update(getImuHeading());
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
            }


            // --- Arcade Drive Power Calculation ---

            // Full power mode: hold Gamepad 1 left trigger. Full forward stick becomes 100% power
            // on all drive motors; turning stays at its normal 40%.
            boolean fullPower = gamepad1.left_trigger > 0.1;

            if (assistActive) {
                // The bumper assist drives; the sticks are ignored. Restart the stick ramp
                // from zero so control returns smoothly when the bumper is released.
                robot.setDrivePower(
                        assistLeft,
                        assistLeft,
                        assistRight,
                        assistRight
                );
                rampedDrive = 0.0;
                rampedTurn = 0.0;
                rampTimer.reset();
                straightHolding = false;
            } else {
                // Left stick Y controls forward/backward
                double drive = -gamepad1.left_stick_y;

                // Right stick X controls turning
                double turn = gamepad1.right_stick_x;

                // Reverse driving direction if enabled
                if (isReversed) {
                    drive *= -1;
                }

                // Straight-drive assist applies when the right stick is centered and you are driving
                boolean wantStraight = Math.abs(gamepad1.right_stick_x)
                        < BiobuzzConstants.STRAIGHT_ASSIST_DEADBAND
                        && Math.abs(gamepad1.left_stick_y) > 0.1;

                // Lower the top speed, and the top turn rate even more
                // Full power mode lifts drive to 100%; turning stays at its normal 40% either way
                if (!fullPower) {
                    drive *= BiobuzzConstants.TELEOP_DRIVE_SCALE;
                }
                turn *= BiobuzzConstants.TELEOP_TURN_SCALE;

                // Ramp drive and turn toward the requested values to avoid tipping
                // (turn uses a much shorter ramp so steering stays crisp)
                double dt = rampTimer.seconds();
                rampTimer.reset();
                double maxDriveStep = dt / BiobuzzConstants.DRIVE_RAMP_TIME_SEC;
                double maxTurnStep = dt / BiobuzzConstants.TURN_RAMP_TIME_SEC;

                rampedDrive = BiobuzzConstants.slew(rampedDrive, drive, maxDriveStep);
                rampedTurn = BiobuzzConstants.slew(rampedTurn, turn, maxTurnStep);
                drive = rampedDrive;
                turn = rampedTurn;

                // Gyro assist: hold the heading the robot had when you started driving straight.
                // A positive error means the robot must turn right, which is a positive turn.
                if (imu != null && wantStraight) {
                    double heading = getImuHeading();
                    if (!straightHolding) {
                        // After a turn, wait until the robot has stopped rotating before locking on,
                        // so it holds the heading it ended up at, not one it was still swinging through
                        double spin = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
                        if (Math.abs(spin) < BiobuzzConstants.STRAIGHT_ASSIST_SETTLE_DPS) {
                            straightHeading = heading;
                            straightHolding = true;
                        }
                    }
                    if (straightHolding) {
                        double error = BiobuzzAutoDrive.normalizeDegrees(straightHeading - heading);
                        turn += Math.max(-BiobuzzConstants.STRAIGHT_ASSIST_MAX,
                                Math.min(BiobuzzConstants.STRAIGHT_ASSIST_MAX,
                                        BiobuzzConstants.STRAIGHT_ASSIST_KP * error));
                    }
                } else {
                    straightHolding = false;
                }

                // Combine forward movement and turning
                double leftPower = (drive + turn);
                double rightPower = (drive - turn);

                // Normalize powers if either side exceeds 1.0
                double max = Math.max(
                        Math.abs(leftPower),
                        Math.abs(rightPower)
                );

                if (max > 1.0) {
                    leftPower /= max;
                    rightPower /= max;
                }

                // Send power to the drivetrain motors
                robot.setDrivePower(
                        leftPower,
                        leftPower,
                        rightPower,
                        rightPower
                );
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

            // Open the gate while the trigger exceeds the threshold
            if (gamepad1.right_trigger
                    > BiobuzzConstants.GATE_TRIGGER_THRESHOLD) {

                robot.setGate(true);

            } else {
                // Close the gate when the trigger is released
                robot.setGate(false);
            }


            // --- Shooter Control ---

            // Start the shooter (CLOSE) on Gamepad 1 D-pad up
            if (gamepad1.dpad_up) {

                targetPreset = BiobuzzConstants.ShooterPreset.CLOSE;

            } else if (gamepad1.dpad_down) {

                // Stop the shooter on Gamepad 1 D-pad down
                targetPreset = BiobuzzConstants.ShooterPreset.STOP;
            }


            // --- Shooter Motor Control ---

            double targetRpm =
                    BiobuzzConstants.getTargetRpm(targetPreset);

            robot.setShooterWheelRpm(targetRpm);

            // Apply voltage compensation to shooter PIDF
            robot.applyShooterPIDFWithVoltageComp();


            // --- Telemetry ---

            telemetry.addData("Drive Mode", "Arcade");
            telemetry.addData(
                    "Orientation",
                    isReversed ? "REVERSED" : "NORMAL"
            );
            telemetry.addData(
                    "Full Power",
                    fullPower ? "ON" : "OFF"
            );

            telemetry.addData(
                    "Intake Status",
                    robot.intakeMotor.getPower() != 0
                            ? (robot.intakeMotor.getPower() > 0
                            ? "FORWARD" : "REVERSE")
                            : "STOPPED"
            );

            telemetry.addData("Gate",
                    gamepad1.right_trigger
                            > BiobuzzConstants.GATE_TRIGGER_THRESHOLD
                            ? "OPEN" : "CLOSED");

            if (pose != null) {
                pose.addPoseTelemetry();
                if (relocalizer != null) {
                    relocalizer.addTelemetry(telemetry);
                }
                telemetry.addData("Bumper Mode",
                        xyAimMode ? "AIM AT GOAL (field point)" : "FIXED HEADING");
                if (xyAimMode && spotAim != null) {
                    telemetry.addData("Aim", spotAim.getStatus());
                }
            } else {
                telemetry.addData("Pose", "Pinpoint not available");
            }

            if (headingHold != null) {
                telemetry.addData("IMU Heading (deg)", "%.1f", getImuHeading());
                telemetry.addData("Heading Hold", headingHold.getStatus());
                telemetry.addData("Straight Assist", straightHolding ? "HOLDING" : "off");
            } else {
                telemetry.addData("Heading Hold", "IMU not available");
            }

            telemetry.addData("Shooter Preset", targetPreset);

            telemetry.addData(
                    "Shooter RPM",
                    "Target: %.0f, Actual: %.0f",
                    targetRpm,
                    robot.getShooterWheelRpm()
            );

            telemetry.addData(
                    "Shooter Ready",
                    robot.isShooterAtSpeed(
                            targetRpm,
                            BiobuzzConstants.SHOOTER_RPM_TOLERANCE
                    ) ? "READY" : "WAITING"
            );

            telemetry.update();
        }

        if (vision != null) {
            vision.close();
        }
    }

    /** IMU yaw in degrees, counterclockwise positive (0 right after the IMU was reset). */
    private double getImuYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /** Field heading from the IMU: 0 faces +Y and it increases turning right, so 90 faces +X. */
    private double getImuHeading() {
        return BiobuzzAutoDrive.normalizeDegrees(-getImuYaw() + imuHeadingOffset);
    }
}
