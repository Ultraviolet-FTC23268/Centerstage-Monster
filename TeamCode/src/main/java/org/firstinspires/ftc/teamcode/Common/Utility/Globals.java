package org.firstinspires.ftc.teamcode.Common.Utility;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Vision.Location;
@Config
public class Globals {
    
    public static Pose error = new Pose(0, 0, 0);
    public static Pose targetPose = new Pose(0, 0, 0);
    public static boolean reached = false;

    public static Location SIDE = Location.CLOSE;
    public static Location ALLIANCE = Location.BLUE;
    public static Location PRELOAD = Location.LEFT;
    public static Location RANDOMIZATION = Location.LEFT;

    public static boolean AUTO = false;
    public static boolean USING_IMU = true;
    public static boolean MANUAL_ENABLED = true;
    public static boolean HAS_AUTO_ERROR = false;
    public static boolean IS_PARKING = false;
    public static boolean USE_WHEEL_FEEDFORWARD = true;

    public static boolean SWERVE_X = false;

    //Lift Positions
    public static int DOWN_POS = 0;
    public static int AUTO_POS = 815;
    public static int ROW1_POS = 850;
    public static int ROW2_POS = 1150;
    public static int ROW3_POS = 1450;
    public static int ROW4_POS = 1800;
    public static int ROW5_POS = 2200;
    public static int ROW6_POS = 2550;
    public static int ROW7_POS = 2550;
    public static int ROW8_POS = 2550;
    public static int ROW9_POS = 2550;
    public static int ROW10_POS = 2550;
    public static int ROW11_POS = 2550;
    public static int MAX_POS = 2550;

    public static int SWING_IN_POS = 325;
    public static double LIFT_ERROR_TOLERANCE = 20;
    public static int LIFT_RESET_OFFSET = 150;

    //Drone Positions
    public static double DRONE_CLOSED = 0.6;
    public static double DRONE_OPEN = 0.25;

    //Purple Latch Positions
    public static double PURPLE_CLOSED = 0.5;
    public static double PURPLE_OPEN = 0;

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

    public static int getTargetIndex() {
        int index = 1;

        if (PRELOAD == Location.RIGHT) index += 0;
        else if (PRELOAD == Location.LEFT) index += 0;

        if (RANDOMIZATION == Location.CENTER) index += 1;
        else if (RANDOMIZATION == Location.RIGHT) index += 2;

        if (ALLIANCE == Location.RED) index += 3;

        System.out.println("CURRENT INDEX");
        System.out.println(index);
//        System.out.println(Range.clip(index, 0, 11));

//        return Range.clip(index, 0, 5);
        return index;
    }

}
