package org.firstinspires.ftc.teamcode.Funny.Utility;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Funny.Drivetrain.geometry.Pose;

@Config
public class Globals {
    public enum Side {
        LEFT,
        RIGHT
    }

    public static int LEFTSIDE_REGION_X = 0;
    public static int LEFTSIDE_REGION_Y = 170;

    public static int RIGHTSIDE_REGION_X = 0;
    public static int RIGHTSIDE_REGION_Y = 170;

    public static int REGION_WIDTH = 65;
    public static int REGION_HEIGHT = 65;

    public static Pose yummypose = new Pose(0, 0, 0);
    public static Pose error = new Pose(0, 0, 0);
    public static Pose targetPose = new Pose(0, 0, 0);
    public static boolean reached = false;

    public static Side SIDE = Side.LEFT;
    public static boolean AUTO = false;
    public static boolean USING_IMU = true;
    public static boolean MANUAL_ENABLED = true;
    public static boolean HAS_AUTO_ERROR = false;
    public static boolean IS_PARKING = false;
    public static boolean USE_WHEEL_FEEDFORWARD = false;

    public static boolean SWERVE_X = false;

    public static int wait1 = 100;
    public static int wait2 = 300;
    public static int wait3 = 50;
    public static int wait4 = 50;
    public static int wait5 = 200;
    public static int wait6 = 500;

    public static int DOWN_POS = 0;
    public static int ROW1_POS = 0;
    public static int ROW2_POS = 0;
    public static int ROW3_POS = 0;
    public static int ROW4_POS = 0;
    public static int ROW5_POS = 0;
    public static int ROW6_POS = 0;
    public static int ROW7_POS = 0;
    public static int ROW8_POS = 0;
    public static int ROW9_POS = 0;
    public static int ROW10_POS = 0;
    public static int ROW11_POS = 0;

    public static double LIFT_MAX_V = 0;
    public static double LIFT_MAX_A = 0;
    public static double LIFT_MAX_A_RETRACT = 0;
    public static double LIFT_MAX_D = 0;

    public static double LIFT_LATCHED = 0;
    public static double LIFT_INTERMEDIATE = 0;
    public static double LIFT_UNLATCHED = 0;

    public static final double LIFT_TICKS_PER_INCH = 0;

    public static double LIFT_MANUAL_FACTOR = 0;
    public static double LIFT_MIN = 0;
    public static double LIFT_MAX = 0;

    public static double LIFT_EXTENDED_TOLERANCE = 0;
    public static double LIFT_ERROR_TOLERANCE = 0;

}
