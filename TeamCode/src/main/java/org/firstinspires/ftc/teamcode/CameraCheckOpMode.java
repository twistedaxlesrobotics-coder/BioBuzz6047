package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

/**
 * Checks the AprilTag camera, and finds where each goal tag is on the field. It never drives the
 * robot or changes the pose.
 *
 * To find a tag's field position: put the robot at a spot you have measured on the field (with the
 * robot's heading right), press A to set the pose to the reset position, or leave the pose as it
 * is if it is already correct, then read the "Tag field" numbers for that tag and copy them into
 * CAM_TAG_X / CAM_TAG_Y in BiobuzzConstants. Do it from more than one spot to check they agree.
 *
 * Once the tag positions are filled in, "Robot from tag" shows where the camera thinks the robot is,
 * and "Error vs pose" shows how far that is from the Pinpoint. That is what a correction would change.
 *
 * Controls (Gamepad 1): A = set the pose to the reset position (PoseStore.RESET_*).
 */
@TeleOp(name = "Camera Check", group = "Tuning")
public class CameraCheckOpMode extends LinearOpMode {

    BiobuzzHardware robot = new BiobuzzHardware();
    BiobuzzAutoDrive pose = null;
    BiobuzzVision vision = null;
    boolean prevA = false;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        try {
            pose = new BiobuzzAutoDrive(this, robot);
            pose.initWithoutCalibration();
            pose.setStartPose(PoseStore.getX(), PoseStore.getY(), PoseStore.getHeading());
        } catch (Exception e) {
            pose = null;
        }

        String visionError = null;
        try {
            vision = new BiobuzzVision(hardwareMap);
        } catch (Exception e) {
            vision = null;
            visionError = e.getMessage();
        }

        telemetry.addData("Pose", pose == null ? "Pinpoint not found" : "OK");
        telemetry.addData("Camera", vision == null ? "NOT FOUND: " + visionError : "OK");
        telemetry.addData(">", "Press START. A = set the pose to the reset position");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (pose != null) {
                if (gamepad1.a && !prevA) {
                    pose.setStartPose(PoseStore.RESET_X, PoseStore.RESET_Y, PoseStore.RESET_HEADING);
                }
                prevA = gamepad1.a;
                pose.update();
                pose.addPoseTelemetry();
            }

            if (vision == null) {
                telemetry.addData("Camera", "not available");
            } else {
                telemetry.addData("Camera State", vision.cameraState());
                telemetry.addData("Tags Seen", vision.seenIds());

                List<BiobuzzVision.Observation> seen = vision.observe();
                for (BiobuzzVision.Observation o : seen) {
                    String name = "Tag " + o.id;
                    telemetry.addData(name + " range / bearing", "%.1f in / %.1f deg", o.range, o.bearing);
                    telemetry.addData(name + " in robot frame", "right %.1f, forward %.1f", o.tagRobotX, o.tagRobotY);

                    if (pose != null) {
                        double h = pose.getHeading();
                        double[] tag = BiobuzzRelocalizer.tagFromRobot(pose.getX(), pose.getY(), h, o);
                        telemetry.addData(name + " Tag field (copy me)", "x %.1f, y %.1f", tag[0], tag[1]);

                        if (BiobuzzRelocalizer.tagKnown(o.id)) {
                            double[] est = BiobuzzRelocalizer.robotFromTag(
                                    BiobuzzRelocalizer.tagX(o.id), BiobuzzRelocalizer.tagY(o.id), h, o);
                            telemetry.addData(name + " Robot from tag", "x %.1f, y %.1f", est[0], est[1]);
                            telemetry.addData(name + " Error vs pose", "dx %.1f, dy %.1f in",
                                    est[0] - pose.getX(), est[1] - pose.getY());
                        } else {
                            telemetry.addData(name + " Robot from tag", "tag position not set in BiobuzzConstants");
                        }
                    }
                }
                if (seen.isEmpty()) {
                    telemetry.addData("Measured tags", "none in view");
                }
            }

            telemetry.update();
        }

        if (vision != null) {
            vision.close();
        }
    }
}
