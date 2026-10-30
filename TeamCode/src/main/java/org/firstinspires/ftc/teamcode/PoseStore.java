package org.firstinspires.ftc.teamcode;

/**
 * PoseStore
 * Carries the robot's field pose from one OpMode to the next (auto to teleop). Values live in
 * static fields, so they last only while the Robot Controller app keeps running.
 *
 * Field coordinates (inches, degrees) are described in BiobuzzAutoDrive.
 */
public class PoseStore {

    /** Where the robot sits when the pose is reset by hand, and when no pose has been saved. */
    public static final double RESET_X = 72.0;
    public static final double RESET_Y = 9.0;
    public static final double RESET_HEADING = 180.0;

    private static boolean hasPose = false;
    private static double x = RESET_X;
    private static double y = RESET_Y;
    private static double heading = RESET_HEADING;

    public static synchronized void save(double xInches, double yInches, double headingDegrees) {
        x = xInches;
        y = yInches;
        heading = headingDegrees;
        hasPose = true;
    }

    /** True if an OpMode has saved a pose since the app started. */
    public static synchronized boolean hasPose() { return hasPose; }

    public static synchronized double getX() { return x; }
    public static synchronized double getY() { return y; }
    public static synchronized double getHeading() { return heading; }
}
