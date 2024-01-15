package org.firstinspires.ftc.teamcode.Common.Utility;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.Pose;
import org.firstinspires.ftc.teamcode.Other.Side;

@Config
public class Globals {
    
    public static Pose error = new Pose(0, 0, 0);
    public static Pose targetPose = new Pose(0, 0, 0);
    public static boolean reached = false;

    public static Side SIDE = Side.LEFT;
    public static Side COLOR = Side.BLUE;
    public static boolean AUTO = false;
    public static boolean USING_IMU = true;
    public static boolean MANUAL_ENABLED = true;
    public static boolean HAS_AUTO_ERROR = false;
    public static boolean IS_PARKING = false;
    public static boolean USE_WHEEL_FEEDFORWARD = true;

    public static boolean SWERVE_X = false;

    //Lift Positions
    public static int DOWN_POS = 0;
    public static int ROW1_POS = 675;
    public static int ROW2_POS = 900;
    public static int ROW3_POS = 1100;
    public static int ROW4_POS = 1375;
    public static int ROW5_POS = 1675;
    public static int ROW6_POS = 1860;
    public static int ROW7_POS = 1860;
    public static int ROW8_POS = 1860;
    public static int ROW9_POS = 1860;
    public static int ROW10_POS = 1860;
    public static int ROW11_POS = 1860;
    public static int MAX_POS = 1860;

    public static int SWING_IN_POS = 325;
    public static double LIFT_ERROR_TOLERANCE = 20;
    public static int LIFT_RESET_OFFSET = 150;

    //Drone Positions
    public static double DRONE_CLOSED = 0.65;
    public static double DRONE_OPEN = 0.25;

    //Move Arm Delays
    public static int FLIP_OUT_DELAY = 100;

    //Eject Delays
    public static int EJECT_DELAY = 500;

    //Reset Arm Delays
    public static int FLIP_IN_DELAY = 250;
    public static int LIFT_DELAY = 800;
    public static int FLIP_AROUND_DELAY = 1000;
    public static int LIFT_SETTLE_DELAY = 600;

    //Score Delays
    public static int SCORE_DROP_DELAY = 250;

    //Unscore Delays
    public static int RESET_DELAY = 350;

    //Intake Delays
    public static int OUTTAKE_DELAY = 1000;

    //Swerve Stuff
    public static double SLOWDOWN_SPEED = 0.5;
    public static boolean SWERVE = true;

}
