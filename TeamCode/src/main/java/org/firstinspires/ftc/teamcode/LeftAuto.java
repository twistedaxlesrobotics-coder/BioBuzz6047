package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * LeftAuto
 * Starts at (-59.5, -14.5) facing heading -90. Backs up to (-46, -14.5) for the opening shot, but holds
 * the shot until SHOT_DELAY_S seconds after START. Then makes one move, backing to the park at
 * (-47, -52). The start and shot are the mirror of RightAuto; the rest is not. Every move runs at the
 * slower SPEED below, and the park is a single attempt that stops when END_TIMEOUT_S runs out.
 */
@Autonomous(name = "LeftAuto", group = "Auto")
public class LeftAuto extends BiobuzzAutoBase {

    /** The shot is not fired until this many seconds after START. */
    private static final double SHOT_DELAY_S = 10.0;

    /** Top power and start-ramp seconds for every move in this auto. Raise the power to go faster. */
    private static final BiobuzzAutoDrive.MoveSpeed SPEED = BiobuzzAutoDrive.MoveSpeed.of(0.45, 0.5);

    private static final double END_X = -47.0;
    private static final double END_Y = -52.0;

    /**
     * The park move is stopped when this many seconds pass, even if the Pinpoint has not reported
     * arriving. Time one full run and set this to the time it takes to reach the park, plus about 0.5.
     */
    private static final double END_TIMEOUT_S = 6.0;

    @Override protected double getStartX() { return BiobuzzConstants.START_LEFT_X; }
    @Override protected double getStartY() { return BiobuzzConstants.START_LEFT_Y; }
    @Override protected double getStartHeading() { return BiobuzzConstants.START_LEFT_HEADING; }

    @Override protected double getShotX() { return -46.0; }
    @Override protected double getShotY() { return -14.5; }

    @Override protected double getEarliestShotSeconds() { return SHOT_DELAY_S; }

    @Override protected BiobuzzAutoDrive.MoveSpeed getMoveSpeed() { return SPEED; }

    /** The opening shot goes to the left hive, so the camera uses the left hive's tags. */
    @Override protected int getCameraSide() { return BiobuzzRelocalizer.SIDE_LEFT; }

    @Override
    protected void runRoute() {
        // One move, one attempt: if the Pinpoint loses distance, the timeout stops the robot, not the wall
        drive.driveToBackward(END_X, END_Y, END_TIMEOUT_S, SPEED);
    }
}
