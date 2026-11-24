package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * BiobuzzVision
 * The AprilTag webcam. Builds the camera and tag processor, and turns each tag the camera sees into
 * its position relative to the ROBOT CENTER, using where the camera is mounted (BiobuzzConstants).
 *
 * The camera points straight back and is tilted up, so the numbers the SDK reports (relative to the
 * camera) are turned into the robot's frame: X to the right, Y forward, in inches, in observe().
 * Creating this throws if the camera is not in the robot configuration, so create it inside a
 * try/catch.
 */
public class BiobuzzVision {

    /** One tag the camera can see right now, with its position relative to the robot center. */
    public static class Observation {
        public int id;
        /** Distance from the camera to the tag, and the bearing to it from the camera's centerline. */
        public double range;
        public double bearing;
        /** Tag position relative to the camera as the SDK reports it: right, forward, up (inches). */
        public double camX, camY, camZ;
        /** Tag position relative to the robot center in the robot's frame: right +, forward + (inches). */
        public double tagRobotX, tagRobotY;
    }

    private final AprilTagProcessor aprilTag;
    private final VisionPortal portal;
    private boolean cameraSettingsApplied = false;

    public BiobuzzVision(HardwareMap map) {
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(
                        BiobuzzConstants.CAM_FX,
                        BiobuzzConstants.CAM_FY,
                        BiobuzzConstants.CAM_CX,
                        BiobuzzConstants.CAM_CY)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(map.get(WebcamName.class, BiobuzzConstants.CAM_NAME))
                .setCameraResolution(new Size(BiobuzzConstants.CAM_WIDTH, BiobuzzConstants.CAM_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Sets manual exposure and gain once the camera is streaming. Call it every loop so the settings
     * go in as soon as the camera starts; it does nothing after that. observe() also calls it.
     */
    public void applyCameraSettings() {
        if (cameraSettingsApplied) return;
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) return;

        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        GainControl gain = portal.getCameraControl(GainControl.class);
        if (exposure != null) {
            exposure.setMode(ExposureControl.Mode.Manual);
            exposure.setExposure((long) BiobuzzConstants.CAM_EXPOSURE_MS, TimeUnit.MILLISECONDS);
        }
        if (gain != null) {
            gain.setGain(BiobuzzConstants.CAM_GAIN);
        }
        cameraSettingsApplied = true;
    }

    /** Every tag the camera currently sees that the SDK can measure, as positions relative to the robot. */
    public List<Observation> observe() {
        applyCameraSettings();

        List<Observation> result = new ArrayList<>();
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null) return result;

        double tilt = Math.toRadians(BiobuzzConstants.CAM_TILT_UP_DEG);
        double cosTilt = Math.cos(tilt);
        double sinTilt = Math.sin(tilt);

        for (AprilTagDetection detection : detections) {
            if (!(detection instanceof AprilTagSingleDetection)) continue;
            AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
            if (single.metadata == null || single.ftcPose == null) continue;

            Observation o = new Observation();
            o.id = single.id;
            o.range = single.ftcPose.range;
            o.bearing = single.ftcPose.bearing;
            o.camX = single.ftcPose.x;
            o.camY = single.ftcPose.y;
            o.camZ = single.ftcPose.z;

            // The SDK gives the tag relative to the camera: x right, y straight out of the lens, z up.
            // Tilting the camera up by the tilt angle, the distance along the floor away from the
            // camera is y*cos(tilt) - z*sin(tilt).
            double groundAway = o.camY * cosTilt - o.camZ * sinTilt;

            // The camera faces BACKWARD, so away from the camera is toward the robot's back, and the
            // camera's right is the robot's left.
            o.tagRobotX = BiobuzzConstants.CAM_MOUNT_X_IN - o.camX;
            o.tagRobotY = BiobuzzConstants.CAM_MOUNT_Y_IN - groundAway;
            result.add(o);
        }
        return result;
    }

    /** The IDs of every tag the camera sees, even ones the SDK cannot measure, for telemetry. */
    public String seenIds() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null || detections.isEmpty()) return "none";
        StringBuilder sb = new StringBuilder();
        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagSingleDetection) {
                if (sb.length() > 0) sb.append(", ");
                sb.append(((AprilTagSingleDetection) detection).id);
            }
        }
        return sb.length() == 0 ? "none" : sb.toString();
    }

    public String cameraState() {
        return String.valueOf(portal.getCameraState());
    }

    public void close() {
        portal.close();
    }
}
