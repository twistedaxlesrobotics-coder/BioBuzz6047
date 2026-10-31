package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * RightAuto
 * Starts at (131.5, 57.5) facing heading 90. Backs up to (118, 57.5) for the opening shot, then backs
 * toward the driver wall to (119, 28) and on to the park at (47, 15). Every move runs at the slower
 * SPEED below, and the park is a single attempt that stops when END_TIMEOUT_S runs out.
 */
@Autonomous(name = "RightAuto", group = "Auto")
public class RightAuto extends BiobuzzAutoBase {

    /** Top power and start-ramp seconds for every move in this auto. Raise the power to go faster. */
    private static final BiobuzzAutoDrive.MoveSpeed SPEED = BiobuzzAutoDrive.MoveSpeed.of(0.45, 0.5);

    private static final double WAYPOINT_1_X = 119.0;
    private static final double WAYPOINT_1_Y = 28.0;
    private static final double WAYPOINT_1_TIMEOUT_S = 6.0;
    private static final double END_X = 47.0;
    private static final double END_Y = 15.0;

    /**
     * The park move is stopped when this many seconds pass, even if the OTOS has not reported
     * arriving. Time one full run and set this to the time it takes to reach x = 47, plus about 0.5.
     */
    private static final double END_TIMEOUT_S = 5.0;

    @Override protected double getStartX() { return 131.5; }
    @Override protected double getStartY() { return 57.5; }
    @Override protected double getStartHeading() { return 90.0; }

    @Override protected double getShotX() { return 118.0; }
    @Override protected double getShotY() { return 57.5; }

    @Override protected BiobuzzAutoDrive.MoveSpeed getMoveSpeed() { return SPEED; }

    @Override
    protected void runRoute() {
        viaBackward(WAYPOINT_1_X, WAYPOINT_1_Y, WAYPOINT_1_TIMEOUT_S, SPEED);

        // One attempt only: if the OTOS loses distance, the timeout stops the robot, not the wall
        drive.driveToBackward(END_X, END_Y, END_TIMEOUT_S, SPEED);
    }
}
