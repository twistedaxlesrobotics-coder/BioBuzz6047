package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Checks the AprilTag camera, and LEARNS where each hive tag is on the field. It never drives the
 * robot or changes the pose.
 *
 * To learn the tags:
 * 1. Put the hive in its normal (up) shooting position.
 * 2. Put the robot at a spot whose position you know exactly, facing the right way, and tell the
 *    software where it is with one of these buttons:
 *      D-pad right = the RIGHT auto start spot,  D-pad left = the LEFT auto start spot
 *      (place the robot exactly as you do before that auto),
 *      A = the reset position (PoseStore.RESET_*),
 *      X = your own spot, typed into CAM_LEARN_X / CAM_LEARN_Y / CAM_LEARN_HEADING.
 *    The software cannot tell where the robot really is, so it is only as accurate as your placement.
 * 3. Aim the camera so it sees the tags, and HOLD B for about two seconds, then let go.
 *    Every visible red-alliance hive is learned and saved on the robot. The SDK may report a hive as
 *    a single named "cluster" (such as "red audience") or as separate tags; both are handled.
 * 4. Do it again from another spot to pick up tags it could not see, or to check them. A tag that is
 *    learned again is replaced.
 *
 * Once tags are learned, "Robot from tag" shows where the camera thinks the robot is, and "Error vs
 * pose" shows how far that is from the Pinpoint. That is what a correction would change.
 *
 * Controls (Gamepad 1): D-pad right / D-pad left / A / X = set the pose to a known spot (see above),
 * hold B = learn the visible tags, left bumper + Y = forget all learned tags.
 */
@TeleOp(name = "Camera Check", group = "Tuning")
public class CameraCheckOpMode extends LinearOpMode {

    /** Frames needed while B is held before a tag counts as learned. */
    private static final int MIN_LEARN_FRAMES = 10;

    BiobuzzHardware robot = new BiobuzzHardware();
    BiobuzzAutoDrive pose = null;
    BiobuzzVision vision = null;

    boolean prevA = false;
    boolean prevX = false;
    boolean prevDpadLeft = false;
    boolean prevDpadRight = false;
    boolean prevB = false;
    boolean prevClear = false;

    // While B is held: tag number or cluster name -> {sum x, sum y, frames}
    final Map<String, double[]> learning = new HashMap<>();
    String learnMessage = "Hold B to learn the visible tags";

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        BiobuzzTagMap.load();

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
        telemetry.addData("Learned tags", BiobuzzTagMap.count());
        telemetry.addData(">", "Press START. D-pad right/left = auto start spot, A = reset position, X = own spot, hold B = learn tags");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (pose != null) {
                if (gamepad1.a && !prevA) {
                    pose.setStartPose(PoseStore.RESET_X, PoseStore.RESET_Y, PoseStore.RESET_HEADING);
                    learnMessage = "Pose set to the reset position";
                }
                if (gamepad1.dpad_right && !prevDpadRight) {
                    pose.setStartPose(BiobuzzConstants.START_RIGHT_X, BiobuzzConstants.START_RIGHT_Y,
                            BiobuzzConstants.START_RIGHT_HEADING);
                    learnMessage = "Pose set to the RIGHT auto start spot";
                }
                if (gamepad1.dpad_left && !prevDpadLeft) {
                    pose.setStartPose(BiobuzzConstants.START_LEFT_X, BiobuzzConstants.START_LEFT_Y,
                            BiobuzzConstants.START_LEFT_HEADING);
                    learnMessage = "Pose set to the LEFT auto start spot";
                }
                if (gamepad1.x && !prevX) {
                    if (Double.isNaN(BiobuzzConstants.CAM_LEARN_X) || Double.isNaN(BiobuzzConstants.CAM_LEARN_Y)) {
                        learnMessage = "Set CAM_LEARN_X / Y / HEADING in BiobuzzConstants to use X";
                    } else {
                        pose.setStartPose(BiobuzzConstants.CAM_LEARN_X, BiobuzzConstants.CAM_LEARN_Y,
                                BiobuzzConstants.CAM_LEARN_HEADING);
                        learnMessage = "Pose set to your own spot";
                    }
                }
                prevA = gamepad1.a;
                prevX = gamepad1.x;
                prevDpadLeft = gamepad1.dpad_left;
                prevDpadRight = gamepad1.dpad_right;
                pose.update();
                pose.addPoseTelemetry();
            }

            // Forget everything (left bumper + Y)
            boolean clear = gamepad1.left_bumper && gamepad1.y;
            if (clear && !prevClear) {
                BiobuzzTagMap.clear();
                boolean saved = BiobuzzTagMap.save();
                learnMessage = saved ? "All learned tags forgotten" : "Forgotten, but the file could not be saved";
            }
            prevClear = clear;

            if (vision == null) {
                telemetry.addData("Camera", "not available");
            } else {
                telemetry.addData("Camera State", vision.cameraState());
                telemetry.addData("Tags Seen", vision.seenIds());

                List<BiobuzzVision.Observation> seen = vision.observe();
                boolean holdingB = gamepad1.b;
                boolean weakView = false;   // a cluster is in view but too little of it to learn from

                for (BiobuzzVision.Observation o : seen) {
                    String name = o.cluster ? "Cluster " + o.key : "Tag " + o.id;
                    telemetry.addData(name + " range / bearing", "%.1f in / %.1f deg", o.range, o.bearing);
                    if (o.cluster) {
                        telemetry.addData(name + " tags found", "%d percent", o.percentFound);
                    }

                    if (pose == null) continue;
                    double h = pose.getHeading();
                    double[] tag = BiobuzzRelocalizer.tagFromRobot(pose.getX(), pose.getY(), h, o);
                    telemetry.addData(name + " field position now", "x %.1f, y %.1f", tag[0], tag[1]);

                    // Collect frames while B is held
                    boolean goodEnough = !o.cluster
                            || o.percentFound >= BiobuzzConstants.CAM_LEARN_MIN_CLUSTER_PERCENT;
                    if (holdingB && BiobuzzRelocalizer.isLearnable(o) && !goodEnough) {
                        weakView = true;
                    }
                    if (holdingB && BiobuzzRelocalizer.isLearnable(o) && goodEnough) {
                        double[] acc = learning.get(o.key);
                        if (acc == null) {
                            acc = new double[3];
                            learning.put(o.key, acc);
                        }
                        acc[0] += tag[0];
                        acc[1] += tag[1];
                        acc[2] += 1;
                    }

                    if (BiobuzzTagMap.known(o.key)) {
                        double[] est = BiobuzzRelocalizer.robotFromTag(
                                BiobuzzTagMap.x(o.key), BiobuzzTagMap.y(o.key), h, o);
                        telemetry.addData(name + " Robot from tag", "x %.1f, y %.1f", est[0], est[1]);
                        telemetry.addData(name + " Error vs pose", "dx %.1f, dy %.1f in",
                                est[0] - pose.getX(), est[1] - pose.getY());
                    } else {
                        telemetry.addData(name + " Robot from tag", "not learned yet");
                    }
                }
                if (seen.isEmpty()) {
                    telemetry.addData("Measured tags", "none in view");
                }

                // B released: keep the tags that were seen for long enough
                if (!holdingB && prevB) {
                    int learned = 0;
                    for (Map.Entry<String, double[]> entry : learning.entrySet()) {
                        double[] acc = entry.getValue();
                        if (acc[2] >= MIN_LEARN_FRAMES) {
                            BiobuzzTagMap.learn(entry.getKey(), acc[0] / acc[2], acc[1] / acc[2]);
                            learned++;
                        }
                    }
                    learning.clear();
                    if (learned == 0) {
                        learnMessage = "No tag was seen long enough. Hold B longer with the tags in view";
                    } else {
                        boolean saved = BiobuzzTagMap.save();
                        learnMessage = learned + " tag(s) learned" + (saved ? " and saved" : ", but the file could not be saved");
                    }
                }
                prevB = holdingB;
                if (holdingB) {
                    learnMessage = weakView
                            ? "Learning... but too little of the hive is in view, so some of it is skipped. Reposition so more tags show"
                            : "Learning... keep the robot still and the tags in view";
                }
            }

            telemetry.addData("Learn", learnMessage);
            telemetry.addData("Learned tags", BiobuzzTagMap.count());
            telemetry.update();
        }

        if (vision != null) {
            vision.close();
        }
    }
}
