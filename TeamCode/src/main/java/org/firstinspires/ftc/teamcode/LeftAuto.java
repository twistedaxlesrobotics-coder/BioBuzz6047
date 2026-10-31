package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * LeftAuto
 * Starts at (12.5, 57.5) facing heading -90. Backs up to (26, 57.5) for the opening shot, but holds
 * the shot until SHOT_DELAY_S seconds after START. Then backs to (25, 10) and on to (37, 10).
 * The start and shot are the mirror of RightAuto; the rest of the route is not.
 */
@Autonomous(name = "LeftAuto", group = "Auto")
public class LeftAuto extends BiobuzzAutoBase {

    /** The shot is not fired until this many seconds after START. */
    private static final double SHOT_DELAY_S = 10.0;

    private static final double WAYPOINT_1_X = 25.0;
    private static final double WAYPOINT_1_Y = 10.0;
    private static final double WAYPOINT_1_TIMEOUT_S = 6.0;
    private static final double END_X = 37.0;
    private static final double END_Y = 10.0;
    private static final double END_TIMEOUT_S = 5.0;

    @Override protected double getStartX() { return 12.5; }
    @Override protected double getStartY() { return 57.5; }
    @Override protected double getStartHeading() { return -90.0; }

    @Override protected double getShotX() { return 26.0; }
    @Override protected double getShotY() { return 57.5; }

    @Override protected double getEarliestShotSeconds() { return SHOT_DELAY_S; }

    @Override
    protected void runRoute() {
        viaBackward(WAYPOINT_1_X, WAYPOINT_1_Y, WAYPOINT_1_TIMEOUT_S);
        parkAt(END_X, END_Y, END_TIMEOUT_S);
    }
}
