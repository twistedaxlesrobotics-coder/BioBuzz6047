package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * LeftAuto
 * Starts at (9, 56) facing heading -90. Backs up to (25, 56) for the opening shot, then backs to
 * (25, 10) and on to (37, 10).
 */
@Autonomous(name = "LeftAuto", group = "Auto")
public class LeftAuto extends BiobuzzAutoBase {

    private static final double WAYPOINT_1_X = 25.0;
    private static final double WAYPOINT_1_Y = 10.0;
    private static final double WAYPOINT_1_TIMEOUT_S = 6.0;
    private static final double END_X = 37.0;
    private static final double END_Y = 10.0;
    private static final double END_TIMEOUT_S = 5.0;

    @Override protected double getStartX() { return 9.0; }
    @Override protected double getStartY() { return 56.0; }
    @Override protected double getStartHeading() { return -90.0; }

    @Override protected double getShotX() { return 25.0; }
    @Override protected double getShotY() { return 56.0; }

    @Override
    protected void runRoute() {
        viaBackward(WAYPOINT_1_X, WAYPOINT_1_Y, WAYPOINT_1_TIMEOUT_S);
        parkAt(END_X, END_Y, END_TIMEOUT_S);
    }
}
