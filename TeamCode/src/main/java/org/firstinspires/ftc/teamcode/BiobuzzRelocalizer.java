package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * BiobuzzRelocalizer
 * Corrects the robot's field X/Y from a goal AprilTag each time the robot shoots.
 *
 * Call request() when a shot starts (the gate opens), and update() every loop after the pose has
 * been updated. While the request is open it waits for the robot to be still, takes several camera
 * frames of the same tag, and works out where the robot must be from where the tag is. If the frames
 * agree and the correction is not suspiciously large, it moves the pose there. Otherwise nothing
 * changes and the status says why.
 *
 * Only X and Y are corrected. The heading comes from the Pinpoint, and is used to turn the tag's
 * position on the robot into a position on the field.
 *
 * The correction stays OFF for any tag whose field position in BiobuzzConstants (CAM_TAG_X/Y) is
 * still NaN.
 */
public class BiobuzzRelocalizer {

    private final BiobuzzVision vision;
    private final BiobuzzAutoDrive drive;

    // The request in progress
    private boolean active = false;
    private long windowStartNs = 0;
    private long windowNs = 0;

    // Samples of one tag
    private final double[] sampleX = new double[BiobuzzConstants.CAM_FRAMES_NEEDED];
    private final double[] sampleY = new double[BiobuzzConstants.CAM_FRAMES_NEEDED];
    private int count = 0;
    private int sampleTagId = -1;
    private long lastSampleNs = 0;

    // Is the robot still? Worked out from how the pose changes between calls.
    private boolean hasPrev = false;
    private double prevX, prevY, prevHeading;
    private long prevNs;
    private double speed = 0;      // inches per second
    private double turnRate = 0;   // degrees per second

    private String status = "READY";
    private double lastDx = 0;
    private double lastDy = 0;

    public BiobuzzRelocalizer(BiobuzzVision vision, BiobuzzAutoDrive drive) {
        this.vision = vision;
        this.drive = drive;
    }

    /** Starts looking for a tag. The camera has windowSeconds to get a fix. */
    public void request(double windowSeconds) {
        if (vision == null) {
            status = "NO CAMERA";
            return;
        }
        active = true;
        windowStartNs = System.nanoTime();
        windowNs = (long) (windowSeconds * 1e9);
        count = 0;
        sampleTagId = -1;
        lastSampleNs = 0;
        status = "LOOKING";
    }

    public boolean isActive() {
        return active;
    }

    public String getStatus() {
        return status;
    }

    /** Call every loop, after the pose has been updated. */
    public void update() {
        if (vision != null) {
            vision.applyCameraSettings();   // exposure and gain, as soon as the camera is streaming
        }

        long now = System.nanoTime();
        double x = drive.getX();
        double y = drive.getY();
        double h = drive.getHeading();

        // Keep track of how fast the robot is moving and turning
        if (hasPrev) {
            double dt = (now - prevNs) / 1e9;
            if (dt > 0.005) {
                double spd = Math.hypot(x - prevX, y - prevY) / dt;
                double rate = Math.abs(BiobuzzAutoDrive.normalizeDegrees(h - prevHeading)) / dt;
                speed += 0.3 * (spd - speed);
                turnRate += 0.3 * (rate - turnRate);
                prevX = x;
                prevY = y;
                prevHeading = h;
                prevNs = now;
            }
        } else {
            prevX = x;
            prevY = y;
            prevHeading = h;
            prevNs = now;
            hasPrev = true;
        }

        if (!active || vision == null) return;

        // Out of time
        if (now - windowStartNs > windowNs) {
            active = false;
            status = count > 0 ? "NO FIX (frames did not settle)" : "NO FIX (no usable tag)";
            return;
        }

        // Wait for the robot to be still, so the camera frame and the pose line up
        if (speed > BiobuzzConstants.CAM_STILL_SPEED_IN_S
                || turnRate > BiobuzzConstants.CAM_STILL_TURN_DPS) {
            status = "WAITING (robot moving)";
            return;
        }

        // Do not count the same camera frame twice
        if ((now - lastSampleNs) < BiobuzzConstants.CAM_SAMPLE_INTERVAL_MS * 1000000L) return;

        // Pick the nearest configured tag whose field position is known
        List<BiobuzzVision.Observation> seen = vision.observe();
        BiobuzzVision.Observation best = null;
        for (BiobuzzVision.Observation o : seen) {
            if (!tagKnown(o.id)) continue;
            if (o.range > BiobuzzConstants.CAM_MAX_RANGE_IN) continue;
            if (best == null || o.range < best.range) best = o;
        }
        if (best == null) {
            status = seen.isEmpty() ? "LOOKING (no tag in view)" : "LOOKING (tag not configured or too far)";
            return;
        }
        lastSampleNs = now;

        // Frames from a different tag do not mix
        if (best.id != sampleTagId) {
            sampleTagId = best.id;
            count = 0;
        }

        double[] est = robotFromTag(tagX(best.id), tagY(best.id), h, best);
        sampleX[count] = est[0];
        sampleY[count] = est[1];
        count++;
        status = "LOOKING (frames " + count + "/" + sampleX.length + ")";
        if (count < sampleX.length) return;

        // Enough frames: do they agree?
        double meanX = 0, meanY = 0;
        for (int i = 0; i < count; i++) {
            meanX += sampleX[i];
            meanY += sampleY[i];
        }
        meanX /= count;
        meanY /= count;
        double spread = 0;
        for (int i = 0; i < count; i++) {
            spread = Math.max(spread, Math.hypot(sampleX[i] - meanX, sampleY[i] - meanY));
        }
        if (spread > BiobuzzConstants.CAM_MAX_SPREAD_IN) {
            count = 0;
            status = String.format("REJECTED (frames disagree by %.1f in)", spread);
            return;   // keep looking until the window ends
        }

        // Is the correction believable?
        double jump = Math.hypot(meanX - x, meanY - y);
        active = false;
        if (jump > BiobuzzConstants.CAM_MAX_JUMP_IN) {
            status = String.format("REJECTED (%.1f in off, too big)", jump);
            return;
        }
        if (jump < BiobuzzConstants.CAM_MIN_CORRECTION_IN) {
            lastDx = 0;
            lastDy = 0;
            status = String.format("OK (already within %.1f in)", jump);
            return;
        }

        double newX = x + BiobuzzConstants.CAM_BLEND * (meanX - x);
        double newY = y + BiobuzzConstants.CAM_BLEND * (meanY - y);
        drive.correctPosition(newX, newY);
        lastDx = newX - x;
        lastDy = newY - y;
        status = String.format("CORRECTED (%.1f, %.1f in)", lastDx, lastDy);
    }

    /** Adds the camera status lines to telemetry (the caller calls telemetry.update()). */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Camera", status);
        if (vision != null) {
            telemetry.addData("Camera State / Tags", "%s / %s", vision.cameraState(), vision.seenIds());
        }
        telemetry.addData("Last Correction", "%.1f, %.1f in", lastDx, lastDy);
    }

    // =========================================================
    //  TAG TABLE AND GEOMETRY
    // =========================================================

    /** True if this tag is one of the goal tags and its field position has been entered. */
    public static boolean tagKnown(int id) {
        for (int i = 0; i < BiobuzzConstants.CAM_TAG_IDS.length; i++) {
            if (BiobuzzConstants.CAM_TAG_IDS[i] == id) {
                return !Double.isNaN(BiobuzzConstants.CAM_TAG_X[i])
                        && !Double.isNaN(BiobuzzConstants.CAM_TAG_Y[i]);
            }
        }
        return false;
    }

    public static double tagX(int id) {
        for (int i = 0; i < BiobuzzConstants.CAM_TAG_IDS.length; i++) {
            if (BiobuzzConstants.CAM_TAG_IDS[i] == id) return BiobuzzConstants.CAM_TAG_X[i];
        }
        return Double.NaN;
    }

    public static double tagY(int id) {
        for (int i = 0; i < BiobuzzConstants.CAM_TAG_IDS.length; i++) {
            if (BiobuzzConstants.CAM_TAG_IDS[i] == id) return BiobuzzConstants.CAM_TAG_Y[i];
        }
        return Double.NaN;
    }

    /**
     * Where the robot is on the field, given where the tag is on the field, the robot's heading, and
     * where the tag is relative to the robot. Heading 0 faces +Y and 90 faces +X, so forward is
     * (sin h, cos h) and right is (cos h, -sin h).
     */
    public static double[] robotFromTag(double tagFieldX, double tagFieldY, double headingDeg,
                                        BiobuzzVision.Observation o) {
        double c = Math.cos(Math.toRadians(headingDeg));
        double s = Math.sin(Math.toRadians(headingDeg));
        double offsetX = o.tagRobotX * c + o.tagRobotY * s;
        double offsetY = -o.tagRobotX * s + o.tagRobotY * c;
        return new double[] {tagFieldX - offsetX, tagFieldY - offsetY};
    }

    /** Where the tag is on the field, given where the robot is and its heading. The reverse of robotFromTag. */
    public static double[] tagFromRobot(double robotX, double robotY, double headingDeg,
                                        BiobuzzVision.Observation o) {
        double c = Math.cos(Math.toRadians(headingDeg));
        double s = Math.sin(Math.toRadians(headingDeg));
        double offsetX = o.tagRobotX * c + o.tagRobotY * s;
        double offsetY = -o.tagRobotX * s + o.tagRobotY * c;
        return new double[] {robotX + offsetX, robotY + offsetY};
    }
}
