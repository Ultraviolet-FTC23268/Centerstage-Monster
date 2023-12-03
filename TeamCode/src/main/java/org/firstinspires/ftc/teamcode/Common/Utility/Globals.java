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
    public static boolean USE_WHEEL_FEEDFORWARD = false;

    public static boolean SWERVE_X = false;
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
    public static double LIFT_ERROR_TOLERANCE = 0;

}
