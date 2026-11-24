package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * BiobuzzRelocalizer
 * Corrects the robot's field X/Y from the hive AprilTags each time the robot shoots.
 *
 * Call request() when a shot starts (the gate opens), saying which side the robot is shooting to,
 * and update() every loop after the pose has been updated. While the request is open it waits for
 * the robot to be still, takes several camera frames, and works out where the robot must be from
 * where the tags are (BiobuzzTagMap). Only the four tags of the hive on the shooting side are used.
 *
 * Safety checks, so a bad reading does nothing instead of moving the robot the wrong way:
 * - The robot must be still.
 * - Tags seen together in one frame must agree on the robot's position, or the frame is skipped
 *   (a hive that is not in its normal up position makes them disagree).
 * - Several frames must agree with each other.
 * - A fix from only one tag may move the robot a little; several agreeing tags may move it more.
 * If any check fails, the position is left alone and the status says why.
 *
 * Only X and Y are corrected. The heading comes from the Pinpoint, and is used to turn a tag's
 * position on the robot into a position on the field.
 */
public class BiobuzzRelocalizer {

    /** Which hive the robot is shooting at. */
    public static final int SIDE_ANY = 0;
    public static final int SIDE_LEFT = 1;
    public static final int SIDE_RIGHT = 2;

    private final BiobuzzVision vision;
    private final BiobuzzAutoDrive drive;

    // The request in progress
    private boolean active = false;
    private long windowStartNs = 0;
    private long windowNs = 0;
    private int side = SIDE_ANY;

    // Frames collected for one fix
    private final double[] sampleX = new double[BiobuzzConstants.CAM_FRAMES_NEEDED];
    private final double[] sampleY = new double[BiobuzzConstants.CAM_FRAMES_NEEDED];
    private int count = 0;
    private int fewestTags = 0;
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
        BiobuzzTagMap.load();
    }

    /** The shooting side for a bumper: 1 = left bumper, 2 = right bumper, anything else = either side. */
    public static int sideFromBumper(int bumper) {
        if (bumper == 1) return SIDE_LEFT;
        if (bumper == 2) return SIDE_RIGHT;
        return SIDE_ANY;
    }

    /** Starts looking for tags. The camera has windowSeconds to get a fix. */
    public void request(double windowSeconds, int shootingSide) {
        if (vision == null) {
            status = "NO CAMERA";
            return;
        }
        active = true;
        windowStartNs = System.nanoTime();
        windowNs = (long) (windowSeconds * 1e9);
        side = shootingSide;
        count = 0;
        fewestTags = 0;
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
            if (!status.startsWith("REJECTED")) {
                status = count > 0 ? "NO FIX (frames did not settle)" : "NO FIX (no usable tag)";
            }
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

        // Where does each usable tag say the robot is?
        int[] allowed = idsForSide(side);
        List<BiobuzzVision.Observation> seen = vision.observe();
        List<double[]> estimates = new ArrayList<>();
        for (BiobuzzVision.Observation o : seen) {
            if (!contains(allowed, o.id)) continue;
            if (!BiobuzzTagMap.known(o.id)) continue;
            if (o.range > BiobuzzConstants.CAM_MAX_RANGE_IN) continue;
            estimates.add(robotFromTag(BiobuzzTagMap.x(o.id), BiobuzzTagMap.y(o.id), h, o));
        }
        if (estimates.isEmpty()) {
            status = seen.isEmpty()
                    ? "LOOKING (no tag in view)"
                    : "LOOKING (no learned tag for this side in view)";
            return;
        }
        lastSampleNs = now;

        // The tags in this frame must agree with each other
        double frameX = 0, frameY = 0;
        for (double[] e : estimates) {
            frameX += e[0];
            frameY += e[1];
        }
        frameX /= estimates.size();
        frameY /= estimates.size();
        if (estimates.size() >= 2) {
            double worst = 0;
            for (double[] e : estimates) {
                worst = Math.max(worst, Math.hypot(e[0] - frameX, e[1] - frameY));
            }
            if (worst > BiobuzzConstants.CAM_MAX_TAG_DISAGREE_IN) {
                count = 0;
                status = String.format("REJECTED (tags disagree by %.1f in)", worst);
                return;   // keep looking until the window ends
            }
        }

        fewestTags = (count == 0) ? estimates.size() : Math.min(fewestTags, estimates.size());
        sampleX[count] = frameX;
        sampleY[count] = frameY;
        count++;
        status = "LOOKING (frames " + count + "/" + sampleX.length + ")";
        if (count < sampleX.length) return;

        // Enough frames: do they agree with each other?
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

        // Is the correction believable? One tag may only move the robot a little.
        double jump = Math.hypot(meanX - x, meanY - y);
        double limit = fewestTags >= 2
                ? BiobuzzConstants.CAM_MAX_JUMP_IN
                : BiobuzzConstants.CAM_SINGLE_TAG_MAX_JUMP_IN;
        active = false;
        if (jump > limit) {
            status = String.format("REJECTED (%.1f in off, limit %.0f with %d tag%s)",
                    jump, limit, fewestTags, fewestTags == 1 ? "" : "s");
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
        status = String.format("CORRECTED (%.1f, %.1f in, %d tag%s)", lastDx, lastDy,
                fewestTags, fewestTags == 1 ? "" : "s");
    }

    /** Adds the camera status lines to telemetry (the caller calls telemetry.update()). */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Camera", status);
        if (vision != null) {
            telemetry.addData("Camera State / Tags", "%s / %s", vision.cameraState(), vision.seenIds());
        }
        telemetry.addData("Last Correction", "%.1f, %.1f in (%d tags learned)",
                lastDx, lastDy, BiobuzzTagMap.count());
    }

    // =========================================================
    //  TAG SETS AND GEOMETRY
    // =========================================================

    /** The tag IDs on the hive for a shooting side, for the current alliance. SIDE_ANY gives both hives. */
    public static int[] idsForSide(int side) {
        int[] left = BiobuzzConstants.ALLIANCE_IS_RED
                ? BiobuzzConstants.CAM_RED_LEFT_TAGS : BiobuzzConstants.CAM_BLUE_LEFT_TAGS;
        int[] right = BiobuzzConstants.ALLIANCE_IS_RED
                ? BiobuzzConstants.CAM_RED_RIGHT_TAGS : BiobuzzConstants.CAM_BLUE_RIGHT_TAGS;
        if (side == SIDE_LEFT) return left;
        if (side == SIDE_RIGHT) return right;

        int[] both = new int[left.length + right.length];
        System.arraycopy(left, 0, both, 0, left.length);
        System.arraycopy(right, 0, both, left.length, right.length);
        return both;
    }

    public static boolean contains(int[] ids, int id) {
        for (int candidate : ids) {
            if (candidate == id) return true;
        }
        return false;
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
