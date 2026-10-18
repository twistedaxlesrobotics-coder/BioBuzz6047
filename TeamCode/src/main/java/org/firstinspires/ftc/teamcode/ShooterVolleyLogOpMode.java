package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * Records the shooter wheel speed through a multi-ball volley so the speed dip per ball can be
 * measured, and lets P and the feedforward be adjusted between volleys.
 *
 * Controls (gamepad 1):
 * - X: Spin the shooter up to the target RPM
 * - B: Stop the shooter
 * - D-pad Right / Left: Target RPM +/- 25
 * - D-pad Up / Down: Feedforward trim +/- 0.02 (starts at 1.464, last year's 1.20 x 1.22)
 * - Right Bumper / Left Bumper: P +/- 25
 * - A: Start a recording window (4 s). Then pull the right trigger to release the balls.
 * - Right Trigger: Move the gate servo (0.0 to 1.0)
 *
 * While idle, the battery-voltage correction on the feedforward is refreshed about once a second,
 * and only when the smoothed voltage has moved by 0.15 V or more. It is frozen while recording and
 * for a short time after the gate was last pulled, so the voltage sag from a volley is not chased.
 *
 * Output: a short summary on the driver station, plus a CSV of every sample saved on the
 * Robot Controller under /sdcard/FIRST/ShooterLogs/ (pull it with Android Studio's Device File
 * Explorer). The CSV header records the P, F, trim and voltage used for that run.
 *
 * Uses the same hardware names as BiobuzzHardware, so the full robot configuration must exist.
 */
@TeleOp(name="Shooter Volley Log", group="Shooter")
public class ShooterVolleyLogOpMode extends LinearOpMode {

    // ---- Recording ----
    private static final int    RECORD_WINDOW_MS = 4000;
    private static final int    MAX_SAMPLES      = 20000;
    private static final double GATE_OPEN_THRESHOLD = 0.05;
    private static final double PRE_WINDOW_MS    = 150.0;
    private static final String LOG_DIR          = "/sdcard/FIRST/ShooterLogs";

    // ---- Tuning limits and steps ----
    private static final double RPM_MIN = 500, RPM_MAX = 5500, RPM_STEP = 25;
    private static final double P_MAX = 1000, P_STEP = 25;
    private static final double TRIM_MIN = 0.5, TRIM_MAX = 2.5, TRIM_STEP = 0.02;
    private static final double DEFAULT_TRIM = 1.20 * 1.22;

    // ---- Voltage correction refresh ----
    private static final long   VOLT_PERIOD_MS = 1000;
    private static final double VOLT_DEADBAND  = 0.15;
    private static final double VOLT_SMOOTH    = 0.3;
    private static final long   GATE_FREEZE_MS = 750;

    // ---- READY ----
    private static final long READY_HOLD_MS = 100;

    BiobuzzHardware robot = new BiobuzzHardware();

    // Tuning state
    private double targetRpm = 2500;
    private boolean shooterOn = false;
    private double pGain = 100.0;
    private double fTrim = DEFAULT_TRIM;
    private double appliedF = 0;
    private double appliedVolts = 12.0;
    private double smoothVolts = 12.0;

    // Recording buffers (preallocated; nothing is allocated inside the loop)
    private final long[]  tUs      = new long[MAX_SAMPLES];
    private final float[] rpmLog   = new float[MAX_SAMPLES];
    private final float[] servoLog = new float[MAX_SAMPLES];
    private final float[] tgtLog   = new float[MAX_SAMPLES];
    private int sampleCount = 0;

    // Parameters captured when a recording starts (written to the CSV header)
    private double recTarget, recP, recF, recTrim, recVolts;

    private final List<String> summary = new ArrayList<>();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // One bulk read per loop; the cache must be cleared by hand every loop in MANUAL mode.
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Replace the PIDF that BiobuzzHardware.init applied with this OpMode's adjustable one.
        smoothVolts = readVoltage();
        applyPidf(smoothVolts);

        telemetry.addData("Status", "Initialized. Press START.");
        telemetry.addData("Target RPM", "%.0f", targetRpm);
        telemetry.update();

        waitForStart();

        boolean prevDpadRight = false, prevDpadLeft = false, prevDpadUp = false, prevDpadDown = false;
        boolean prevRb = false, prevLb = false, prevA = false, prevX = false;

        boolean recording = false;
        long recStartNs = 0;

        boolean pidfDirty = false;
        boolean forceVoltRefresh = false;
        long lastVoltCheckMs = 0;
        long lastGateActiveMs = -GATE_FREEZE_MS;
        long lastTelemetryMs = 0;
        long readySinceMs = -1;

        double lastCmdRpm = -1;
        double lastServoCmd = 0.0;

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }
            long nowNs = System.nanoTime();
            long nowMs = nowNs / 1000000L;

            // --- Gate servo (only send when the value actually changes) ---
            double trigger = gamepad1.right_trigger;
            if (trigger > GATE_OPEN_THRESHOLD) {
                lastGateActiveMs = nowMs;
            }
            if (Math.abs(trigger - lastServoCmd) > 0.004 || (trigger == 0.0 && lastServoCmd != 0.0)) {
                robot.shooter_trigger(trigger);
                lastServoCmd = trigger;
            }

            // --- Button edges ---
            boolean dRight = gamepad1.dpad_right, dLeft = gamepad1.dpad_left;
            boolean dUp = gamepad1.dpad_up, dDown = gamepad1.dpad_down;
            boolean rb = gamepad1.right_bumper, lb = gamepad1.left_bumper;
            boolean aBtn = gamepad1.a, xBtn = gamepad1.x;

            if (!recording) {
                if (dRight && !prevDpadRight) targetRpm = Math.min(RPM_MAX, targetRpm + RPM_STEP);
                if (dLeft && !prevDpadLeft)   targetRpm = Math.max(RPM_MIN, targetRpm - RPM_STEP);
                if (dUp && !prevDpadUp)   { fTrim = Math.min(TRIM_MAX, fTrim + TRIM_STEP); pidfDirty = true; }
                if (dDown && !prevDpadDown) { fTrim = Math.max(TRIM_MIN, fTrim - TRIM_STEP); pidfDirty = true; }
                if (rb && !prevRb) { pGain = Math.min(P_MAX, pGain + P_STEP); pidfDirty = true; }
                if (lb && !prevLb) { pGain = Math.max(0.0, pGain - P_STEP); pidfDirty = true; }
            }
            if (xBtn && !prevX && !shooterOn) {
                forceVoltRefresh = true;   // fresh voltage reading when the shooter is started
            }
            if (xBtn) shooterOn = true;
            if (gamepad1.b) shooterOn = false;   // B always works, even while recording

            boolean startRecording = aBtn && !prevA && !recording && shooterOn;

            prevDpadRight = dRight; prevDpadLeft = dLeft; prevDpadUp = dUp; prevDpadDown = dDown;
            prevRb = rb; prevLb = lb; prevA = aBtn; prevX = xBtn;

            // --- Shooter command (only send when it changes) ---
            double cmdRpm = shooterOn ? targetRpm : 0.0;
            if (cmdRpm != lastCmdRpm) {
                robot.setShooterWheelRpm(cmdRpm);
                lastCmdRpm = cmdRpm;
            }

            // --- PIDF / voltage refresh: never while recording or just after the gate was pulled ---
            boolean frozen = recording || (nowMs - lastGateActiveMs) < GATE_FREEZE_MS;
            if (!frozen) {
                if (pidfDirty) {
                    applyPidf(smoothVolts);
                    pidfDirty = false;
                } else if (forceVoltRefresh || nowMs - lastVoltCheckMs >= VOLT_PERIOD_MS) {
                    double v = readVoltage();
                    smoothVolts = forceVoltRefresh ? v : smoothVolts + VOLT_SMOOTH * (v - smoothVolts);
                    lastVoltCheckMs = nowMs;
                    if (forceVoltRefresh || Math.abs(smoothVolts - appliedVolts) >= VOLT_DEADBAND) {
                        applyPidf(smoothVolts);
                    }
                    forceVoltRefresh = false;
                }
            }

            // --- Start a recording window ---
            if (startRecording) {
                recTarget = cmdRpm; recP = pGain; recF = appliedF; recTrim = fTrim; recVolts = appliedVolts;
                sampleCount = 0;
                recording = true;
                gamepad1.rumble(250);
                telemetry.addData("RECORDING", "%d ms - pull the trigger now", RECORD_WINDOW_MS);
                telemetry.update();
                recStartNs = System.nanoTime();
                nowNs = recStartNs;
            }

            // --- Sample ---
            double rpm = robot.getShooterWheelRpm();

            if (recording) {
                long elapsedUs = (nowNs - recStartNs) / 1000L;
                if (sampleCount < MAX_SAMPLES) {
                    tUs[sampleCount]      = elapsedUs;
                    rpmLog[sampleCount]   = (float) rpm;
                    servoLog[sampleCount] = (float) lastServoCmd;
                    tgtLog[sampleCount]   = (float) cmdRpm;
                    sampleCount++;
                }
                if (elapsedUs >= RECORD_WINDOW_MS * 1000L || sampleCount >= MAX_SAMPLES) {
                    recording = false;
                    gamepad1.rumble(250);
                    buildSummary();
                    String saved = writeCsv();
                    summary.add("Saved: " + saved);
                }
            }

            // --- READY: inside the tolerance band for READY_HOLD_MS ---
            boolean inBand = cmdRpm >= 1.0
                    && Math.abs(rpm - cmdRpm) <= BiobuzzConstants.SHOOTER_RPM_TOLERANCE;
            if (inBand) {
                if (readySinceMs < 0) readySinceMs = nowMs;
            } else {
                readySinceMs = -1;
            }
            boolean ready = readySinceMs >= 0 && (nowMs - readySinceMs) >= READY_HOLD_MS;

            // --- Telemetry (idle only, throttled, so it never slows a recording) ---
            if (!recording && nowMs - lastTelemetryMs >= 100) {
                lastTelemetryMs = nowMs;
                telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
                telemetry.addData("Target / Actual RPM", "%.0f / %.0f", targetRpm, rpm);
                telemetry.addData("Ready", ready ? "YES" : "WAITING");
                telemetry.addData("P | F trim", "%.0f | %.3f", pGain, fTrim);
                telemetry.addData("F applied | Volts", "%.2f | %.2f V", appliedF, appliedVolts);
                telemetry.addLine("X on  B off  A record  Dpad L/R rpm  U/D F trim  Bumpers P");
                for (String line : summary) {
                    telemetry.addLine(line);
                }
                telemetry.update();
            }
        }

        robot.setShooterWheelRpm(0);
    }

    /** Sets the velocity PIDF with the feedforward scaled by trim and battery voltage. */
    private void applyPidf(double volts) {
        double kFNom = 32767.0
                / ((BiobuzzHardware.MOTOR_FREE_RPM * BiobuzzHardware.TICKS_PER_MOTOR_REV) / 60.0);
        double comp = Math.max(0.85, Math.min(1.25, 12.0 / volts));
        appliedF = kFNom * fTrim * comp;
        appliedVolts = volts;
        robot.shooterMotor.setVelocityPIDFCoefficients(pGain, 0.0, 0.0, appliedF);
    }

    private double readVoltage() {
        double voltage = 12.0;
        for (VoltageSensor vs : hardwareMap.getAll(VoltageSensor.class)) {
            double v = vs.getVoltage();
            if (v > 1.0) {
                voltage = v;
                break;
            }
        }
        return Math.max(10.5, voltage);
    }

    /** Three-point moving average of the logged RPM. */
    private double smoothed(int i) {
        int lo = Math.max(0, i - 1);
        int hi = Math.min(sampleCount - 1, i + 1);
        double sum = 0;
        for (int k = lo; k <= hi; k++) sum += rpmLog[k];
        return sum / (hi - lo + 1);
    }

    /** Fills {@link #summary} from the samples just recorded. */
    private void buildSummary() {
        summary.clear();
        if (sampleCount < 5) {
            summary.add("Too few samples recorded.");
            return;
        }
        double durMs = tUs[sampleCount - 1] / 1000.0;
        summary.add(String.format(Locale.US, "Samples: %d in %.0f ms (%.0f Hz)",
                sampleCount, durMs, sampleCount * 1000.0 / durMs));

        int gate = -1;
        for (int i = 0; i < sampleCount; i++) {
            if (servoLog[i] >= GATE_OPEN_THRESHOLD) {
                gate = i;
                break;
            }
        }
        if (gate < 0) {
            summary.add("Gate never opened.");
            return;
        }
        double gateMs = tUs[gate] / 1000.0;

        // Speed and noise over the PRE_WINDOW_MS before the gate opened
        double sum = 0, sumSq = 0;
        int cnt = 0;
        for (int i = gate; i >= 0; i--) {
            if (gateMs - tUs[i] / 1000.0 > PRE_WINDOW_MS) break;
            sum += rpmLog[i];
            sumSq += (double) rpmLog[i] * rpmLog[i];
            cnt++;
        }
        double pre = sum / cnt;
        double noise = cnt > 4 ? Math.sqrt(Math.max(0.0, sumSq / cnt - pre * pre)) : 0.0;

        summary.add(String.format(Locale.US, "Gate opened at %.0f ms", gateMs));
        summary.add(String.format(Locale.US, "Before gate: %.0f RPM (target %.0f, %+.1f%%), noise +/-%.0f",
                pre, recTarget, 100.0 * (pre - recTarget) / recTarget, noise));

        // Find dips: a fall of at least dipThresh below the running peak, ended by a rise of riseThresh
        double dipThresh = Math.max(0.010 * pre, 3.0 * noise);
        double riseThresh = Math.max(0.007 * pre, 2.0 * noise);
        List<double[]> dips = new ArrayList<>();
        double peak = pre, minV = 0, minT = 0;
        boolean inDip = false;
        double lowest = pre, lowestT = gateMs;
        int lowestIdx = gate;
        for (int i = gate; i < sampleCount; i++) {
            double v = smoothed(i);
            double t = tUs[i] / 1000.0;
            if (v < lowest) {
                lowest = v;
                lowestT = t;
                lowestIdx = i;
            }
            if (!inDip) {
                if (v > peak) peak = v;
                if (v < peak - dipThresh) {
                    inDip = true;
                    minV = v;
                    minT = t;
                }
            } else {
                if (v < minV) {
                    minV = v;
                    minT = t;
                }
                if (v > minV + riseThresh) {
                    dips.add(new double[]{peak, minV, minT});
                    inDip = false;
                    peak = v;
                }
            }
        }
        if (inDip) dips.add(new double[]{peak, minV, minT});

        summary.add("Dips found: " + dips.size());
        for (int d = 0; d < dips.size() && d < 5; d++) {
            double[] dip = dips.get(d);
            summary.add(String.format(Locale.US, "  #%d  -%.1f%%  (%.0f -> %.0f)  at %.0f ms",
                    d + 1, 100.0 * (dip[0] - dip[1]) / dip[0], dip[0], dip[1], dip[2]));
        }

        summary.add(String.format(Locale.US, "Lowest: %.0f RPM (%.1f%% below pre-gate) at %.0f ms",
                lowest, 100.0 * (lowest - pre) / pre, lowestT));

        double recoverMs = -1;
        for (int i = lowestIdx; i < sampleCount; i++) {
            if (smoothed(i) >= 0.98 * pre) {
                recoverMs = tUs[i] / 1000.0 - lowestT;
                break;
            }
        }
        if (recoverMs >= 0) {
            summary.add(String.format(Locale.US, "Back within 2%% of pre-gate speed %.0f ms after the lowest point", recoverMs));
        } else {
            summary.add("Not back within 2% of pre-gate speed by the end of the window");
        }
        summary.add(String.format(Locale.US, "End of window: %.0f RPM", smoothed(sampleCount - 1)));
    }

    /** Writes every sample to a CSV on the Robot Controller and returns the file name (or an error). */
    private String writeCsv() {
        File dir = new File(LOG_DIR);
        if (!dir.exists()) {
            dir.mkdirs();
        }
        String name = "volley_"
                + new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date()) + ".csv";
        File file = new File(dir, name);

        StringBuilder sb = new StringBuilder(sampleCount * 28 + 400);
        sb.append(String.format(Locale.US, "# target_rpm,%.0f\n", recTarget));
        sb.append(String.format(Locale.US, "# P,%.1f\n", recP));
        sb.append(String.format(Locale.US, "# F_applied,%.3f\n", recF));
        sb.append(String.format(Locale.US, "# F_trim,%.3f\n", recTrim));
        sb.append(String.format(Locale.US, "# volts_used,%.2f\n", recVolts));
        sb.append("t_ms,rpm,servo,target_rpm\n");
        for (int i = 0; i < sampleCount; i++) {
            sb.append(String.format(Locale.US, "%.2f,%.1f,%.3f,%.0f\n",
                    tUs[i] / 1000.0, rpmLog[i], servoLog[i], tgtLog[i]));
        }

        try (FileWriter writer = new FileWriter(file)) {
            writer.write(sb.toString());
        } catch (IOException e) {
            return "WRITE FAILED: " + e.getMessage();
        }
        return name;
    }
}
