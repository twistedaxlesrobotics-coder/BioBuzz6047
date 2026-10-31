package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * RightAuto
 * Starts at (135, 56) facing heading 90. Backs up to (119, 56) for the opening shot, then backs to
 * (119, 10) and on to (47, 10).
 */
@Autonomous(name = "RightAuto", group = "Auto")
public class RightAuto extends BiobuzzAutoBase {

    private static final double WAYPOINT_1_X = 119.0;
    private static final double WAYPOINT_1_Y = 22.0;
    private static final double WAYPOINT_1_TIMEOUT_S = 6.0;
    private static final double END_X = 47.0;
    private static final double END_Y = 15.0;
    private static final double END_TIMEOUT_S = 8.0;

    @Override protected double getStartX() { return 131.5; }
    @Override protected double getStartY() { return 57.5; }
    @Override protected double getStartHeading() { return 90.0; }

    @Override protected double getShotX() { return 118.0; }
    @Override protected double getShotY() { return 57.5; }

    @Override
    protected void runRoute() {
        viaBackward(WAYPOINT_1_X, WAYPOINT_1_Y, WAYPOINT_1_TIMEOUT_S);
        parkAt(END_X, END_Y, END_TIMEOUT_S);
    }
}
