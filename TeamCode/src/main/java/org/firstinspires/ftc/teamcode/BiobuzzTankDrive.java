
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    @Override
    public void runOpMode() {

        // Initialize robot hardware
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Drive Mode", "Arcade");
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();

        // Main robot control loop
        while (opModeIsActive()) {

            // --- Toggle Front/Back Orientation (D-pad Right) ---

            if (gamepad1.dpad_right && !prevDpadRight) {
                isReversed = !isReversed;
            }

            prevDpadRight = gamepad1.dpad_right;


            // --- Arcade Drive Power Calculation ---

            double driveScale = 1.0;

            // Slow mode: hold Gamepad 1 left trigger
            if (gamepad1.left_trigger > 0.1) {
                driveScale = 0.25;
            }

            // Left stick Y controls forward/backward
            double drive = -gamepad1.left_stick_y;

            // Right stick X controls turning
            double turn = gamepad1.right_stick_x;

            // Reverse driving direction if enabled
            if (isReversed) {
                drive *= -1;
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
                    gamepad2.right_trigger
                            > BiobuzzConstants.GATE_TRIGGER_THRESHOLD
                            ? "OPEN" : "CLOSED");

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
}
