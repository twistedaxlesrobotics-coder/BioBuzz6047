package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * OpMode for tuning and testing the shooter motor and trigger servo.
 * Controls:
 * - X: Start motor at the current target RPM
 * - B: Stop motor (set RPM to 0)
 * - D-pad Right: Increase target RPM by 25
 * - D-pad Left: Decrease target RPM by 25
 * - Right Trigger: Move shooter trigger servo (0.0 to 1.0)
 */
@TeleOp(name="Shooter Tuning", group="Shooter")
public class ShooterTuningOpMode extends LinearOpMode {

    // Hardware class
    BiobuzzHardware robot = new BiobuzzHardware();

    // Tuning state
    double targetRpm = 2500;
    boolean isRunning = false;

    // Edge detection for D-pad
    boolean prevDpadRight = false;
    boolean prevDpadLeft = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized. Target RPM: %.0f", targetRpm);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            
            // --- Target RPM Adjustments ---
            
            // Increase RPM (D-pad Right)
            if (gamepad1.dpad_right && !prevDpadRight) {
                targetRpm += 25;
            }
            prevDpadRight = gamepad1.dpad_right;

            // Decrease RPM (D-pad Left)
            if (gamepad1.dpad_left && !prevDpadLeft) {
                targetRpm -= 25;
            }
            prevDpadLeft = gamepad1.dpad_left;

            // --- Motor State Control ---

            // Start motor (X)
            if (gamepad1.x) {
                isRunning = true;
            }

            // Stop motor (B)
            if (gamepad1.b) {
                isRunning = false;
            }

            // --- Shooter Trigger Control ---
            // Maps the analog trigger value (0.0 - 1.0) directly to the servo position
            robot.shooter_trigger(gamepad1.right_trigger);

            // --- Motor Output ---

            if (isRunning) {
                robot.setShooterWheelRpm(targetRpm);
            } else {
                robot.setShooterWheelRpm(0);
            }

            // Update PIDF with current battery voltage
            robot.applyShooterPIDFWithVoltageComp();

            // --- Telemetry ---
            telemetry.addData("Motor State", isRunning ? "RUNNING" : "STOPPED");
            telemetry.addData("Target RPM", "%.0f", targetRpm);
            telemetry.addData("Actual RPM", "%.0f", robot.getShooterWheelRpm());
            telemetry.addData("Trigger Pos", "%.2f", (robot.triggerServo != null) ? robot.triggerServo.getPosition() : 0.0);
            // Using the global constant for consistency
            telemetry.addData("Ready", robot.isShooterAtSpeed(targetRpm, BiobuzzConstants.SHOOTER_RPM_TOLERANCE) ? "YES" : "WAITING");
            telemetry.update();
        }
    }
}
