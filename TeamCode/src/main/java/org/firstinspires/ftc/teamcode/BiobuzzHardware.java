package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Hardware configuration class for Biobuzz robot.
 */
public class BiobuzzHardware {
    // Drivetrain motors
    public DcMotor tankLeftFront = null;
    public DcMotor tankLeftBack = null;
    public DcMotor tankRightFront = null;
    public DcMotor tankRightBack = null;

    // Intake motor
    public DcMotor intakeMotor = null;

    // Shooter motor
    public DcMotorEx shooterMotor = null;

    // Trigger servo
    public Servo triggerServo = null;

    // Ported Constants from Decode
    public static final double MOTOR_FREE_RPM = 6000.0;
    public static final double TICKS_PER_MOTOR_REV = 28;
    public static final double MOTOR_TO_WHEEL = 1.0; 

    // Shooter PIDF Tuning
    public static final double SHOOTER_KP = 100.0;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.0;
    public static final double SHOOTER_KF_SCALE = 1.22; 
    private static final double KF_MULT = 1.20;

    private HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Initialize Motors
        tankLeftFront  = hwMap.get(DcMotor.class, "tankLeftFront");
        tankLeftBack   = hwMap.get(DcMotor.class, "tankLeftBack");
        tankRightFront = hwMap.get(DcMotor.class, "tankRightFront");
        tankRightBack  = hwMap.get(DcMotor.class, "tankRightBack");
        intakeMotor    = hwMap.get(DcMotor.class, "intakeMotor");
        shooterMotor   = hwMap.get(DcMotorEx.class, "shooterMotor");
        triggerServo   = hwMap.get(Servo.class, "triggerServo");

        // Drivetrain: Left REVERSE, Right FORWARD (for external gearing)
        tankLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        tankLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        tankRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        tankRightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Intake and Shooter
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Servo Initial Position
        if (triggerServo != null) triggerServo.setPosition(0);

        // Reset all
        setDrivePower(0, 0, 0, 0);
        intakeMotor.setPower(0);
        shooterMotor.setPower(0);

        applyShooterPIDFWithVoltageComp();
    }

    public void setDrivePower(double lf, double lb, double rf, double rb) {
        tankLeftFront.setPower(lf);
        tankLeftBack.setPower(lb);
        tankRightFront.setPower(rf);
        tankRightBack.setPower(rb);
    }

    public void setDriveMode(DcMotor.RunMode mode) {
        tankLeftFront.setMode(mode);
        tankLeftBack.setMode(mode);
        tankRightFront.setMode(mode);
        tankRightBack.setMode(mode);
    }

    public void setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        tankLeftFront.setZeroPowerBehavior(behavior);
        tankLeftBack.setZeroPowerBehavior(behavior);
        tankRightFront.setZeroPowerBehavior(behavior);
        tankRightBack.setZeroPowerBehavior(behavior);
    }

    /**
     * shooter_trigger
     * Maps input (typically from an analog trigger 0-1) to servo position.
     */
    public void shooter_trigger(double position) {
        if (triggerServo != null) {
            triggerServo.setPosition(position);
        }
    }

    public void applyShooterPIDFWithVoltageComp() {
        if (shooterMotor == null) return;
        double voltage = readBatteryVoltage();
        double kF_nom = (32767.0 / ((MOTOR_FREE_RPM * TICKS_PER_MOTOR_REV) / 60.0)) * KF_MULT;
        double comp = Math.max(0.85, Math.min(1.25, 12.0 / voltage));
        shooterMotor.setVelocityPIDFCoefficients(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, kF_nom * comp * SHOOTER_KF_SCALE);
    }

    private double readBatteryVoltage() {
        double voltage = 12.0;
        if (hwMap != null) {
            for (VoltageSensor vs : hwMap.getAll(VoltageSensor.class)) {
                double v = vs.getVoltage();
                if (v > 1.0) { voltage = v; break; }
            }
        }
        return Math.max(10.5, voltage);
    }

    public void setShooterWheelRpm(double wheelRpm) {
        shooterMotor.setVelocity((Math.abs(wheelRpm) * TICKS_PER_MOTOR_REV) / 60.0);
    }

    public double getShooterWheelRpm() {
        return (shooterMotor.getVelocity() * 60.0) / TICKS_PER_MOTOR_REV;
    }

    public boolean isShooterAtSpeed(double targetRpm, double tolerance) {
        if (Math.abs(targetRpm) < 1.0) return false;
        return Math.abs(Math.abs(getShooterWheelRpm()) - Math.abs(targetRpm)) <= tolerance;
    }
}
