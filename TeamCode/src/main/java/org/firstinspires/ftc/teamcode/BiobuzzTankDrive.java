package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Biobuzz Tank Drive", group="Linear Opmode")
public class BiobuzzTankDrive extends LinearOpMode {

    // Declare OpMode members
    BiobuzzHardware robot = new BiobuzzHardware();
    
    // Shooter state
    BiobuzzConstants.ShooterPreset targetPreset = BiobuzzConstants.ShooterPreset.STOP;

    // Drive state variables
    boolean isArcadeMode = false;
    boolean isReversed = false;

    // Button state for toggling
    boolean prevDpadLeft = false;
    boolean prevDpadRight = false;

    @Override
    public void runOpMode() {
        // Initialize the hardware
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // --- Toggle Logic ---

            // Toggle between Tank and Split Arcade (D-pad Left)
            if (gamepad1.dpad_left && !prevDpadLeft) {
                isArcadeMode = !isArcadeMode;
            }
            prevDpadLeft = gamepad1.dpad_left;

            // Toggle Front/Back orientation (D-pad Right)
            if (gamepad1.dpad_right && !prevDpadRight) {
                isReversed = !isReversed;
            }
            prevDpadRight = gamepad1.dpad_right;

            // --- Power Calculation ---

            double leftPower;
            double rightPower;
            double driveScale = 1.0;

            // Slow mode check (Left Trigger)
            if (gamepad1.left_trigger > 0.1) {
                driveScale = 0.25;
            }

            if (isArcadeMode) {
                // Split Arcade Mode: Left stick Y is forward/back, Right stick X is turning
                double drive = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;
                if (isReversed) drive *= -1;
                leftPower = (drive + turn) * driveScale;
                rightPower = (drive - turn) * driveScale;
            } else {
                // Regular Tank Drive: Left stick Y is left side, Right stick Y is right side
                double left = -gamepad1.left_stick_y;
                double right = -gamepad1.right_stick_y;
                if (isReversed) {
                    leftPower = -right * driveScale;
                    rightPower = -left * driveScale;
                } else {
                    leftPower = left * driveScale;
                    rightPower = right * driveScale;
                }
            }

            // Normalize powers if they exceed 1.0
            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            // Output to drivetrain motors
            robot.setDrivePower(leftPower, leftPower, rightPower, rightPower);

            // --- Intake Control ---
            if (gamepad1.b) {
                robot.intakeMotor.setPower(1.0);
            } else if (gamepad1.y) {
                robot.intakeMotor.setPower(-1.0);
            } else {
                robot.intakeMotor.setPower(0.0);
            }

            // --- Gate Servo (Right Trigger) ---
            // Gate is open while the trigger is held, and closes when it is released
            robot.setGate(gamepad1.right_trigger > BiobuzzConstants.GATE_TRIGGER_THRESHOLD);

            // --- Shooter Control (mimicking Decode behavior) ---
            if (gamepad1.x) {
                targetPreset = BiobuzzConstants.ShooterPreset.CLOSE;
            } else if (gamepad1.a) {
                targetPreset = BiobuzzConstants.ShooterPreset.MID;
            } else if (gamepad1.left_bumper) {
                targetPreset = BiobuzzConstants.ShooterPreset.STOP;
            }

            double targetRpm = BiobuzzConstants.getTargetRpm(targetPreset);
            robot.setShooterWheelRpm(targetRpm);
            
            // Periodically apply voltage comp to maintain steady RPM as battery drops
            robot.applyShooterPIDFWithVoltageComp();

            // Telemetry
            telemetry.addData("Drive Mode", isArcadeMode ? "Split Arcade" : "Regular Tank");
            telemetry.addData("Orientation", isReversed ? "REVERSED" : "NORMAL");
            telemetry.addData("Slow Mode", driveScale < 1.0 ? "ON" : "OFF");
            telemetry.addData("Intake Status", robot.intakeMotor.getPower() != 0 ? (robot.intakeMotor.getPower() > 0 ? "FORWARD" : "REVERSE") : "STOPPED");
            telemetry.addData("Shooter Preset", targetPreset);
            telemetry.addData("Shooter RPM", "Target: %.0f, Actual: %.0f", targetRpm, robot.getShooterWheelRpm());
            // Fixed: Added SHOOTER_RPM_TOLERANCE argument to match BiobuzzHardware definition
            telemetry.addData("Shooter Ready", robot.isShooterAtSpeed(targetRpm, BiobuzzConstants.SHOOTER_RPM_TOLERANCE) ? "READY" : "WAITING");
            telemetry.update();
        }
    }
}
