
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

    // Field position from the OTOS (null if the sensor is not available)
    BiobuzzAutoDrive pose = null;
    boolean prevPoseReset = false;

    // Heading hold (Gamepad 1 bumpers): 0 = off, 1 = left, 2 = right
    BiobuzzHeadingHold headingHold = null;
    int assistSelection = 0;
    boolean prevAimLocked = false;

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
        } catch (Exception e) {
            pose = null;
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


            // --- Field Pose (OTOS) ---

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
            }


            // --- Heading Hold (Gamepad 1 bumpers) ---
            // Hold the right bumper to turn to the right heading, or the left bumper for the left
            // heading, and keep it there. The sticks are ignored while it is held; release to drive.

            boolean assistActive = false;
            if (headingHold != null) {
                boolean wantLeft = gamepad1.left_bumper && !gamepad1.right_bumper;
                boolean wantRight = gamepad1.right_bumper && !gamepad1.left_bumper;
                int wanted = wantLeft ? 1 : (wantRight ? 2 : 0);

                if (wanted != assistSelection) {
                    assistSelection = wanted;
                    if (wanted == 1) {
                        headingHold.start(BiobuzzConstants.HEADING_HOLD_LEFT_DEG);
                    } else if (wanted == 2) {
                        headingHold.start(BiobuzzConstants.HEADING_HOLD_RIGHT_DEG);
                    } else {
                        headingHold.stop();
                    }
                }

                if (wanted != 0) {
                    headingHold.update(getImuHeading());
                    assistActive = true;
                }

                // Buzz once when the robot is on the heading and settled
                boolean locked = headingHold.isLocked();
                if (locked && !prevAimLocked) {
                    gamepad1.rumble(0.6, 0.6, 300);
                }
                prevAimLocked = locked;
            }


            // --- Arcade Drive Power Calculation ---

            double driveScale = 1.0;

            // Slow mode: hold Gamepad 1 left trigger
            if (gamepad1.left_trigger > 0.1) {
                driveScale = 0.25;
            }

            if (assistActive) {
                // The heading hold drives; the sticks are ignored. Restart the stick ramp
                // from zero so control returns smoothly when the bumper is released.
                robot.setDrivePower(
                        headingHold.leftPower,
                        headingHold.leftPower,
                        headingHold.rightPower,
                        headingHold.rightPower
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
                drive *= BiobuzzConstants.TELEOP_DRIVE_SCALE;
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
                        straightHeading = heading;
                        straightHolding = true;
                    }
                    double error = BiobuzzAutoDrive.normalizeDegrees(straightHeading - heading);
                    turn += Math.max(-BiobuzzConstants.STRAIGHT_ASSIST_MAX,
                            Math.min(BiobuzzConstants.STRAIGHT_ASSIST_MAX,
                                    BiobuzzConstants.STRAIGHT_ASSIST_KP * error));
                } else {
                    straightHolding = false;
                }

                // Combine forward movement and turning
                double leftPower = (drive + turn) * driveScale;
                double rightPower = (drive - turn) * driveScale;

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
                    "Slow Mode",
                    driveScale < 1.0 ? "ON" : "OFF"
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
            } else {
                telemetry.addData("Pose", "OTOS not available");
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
