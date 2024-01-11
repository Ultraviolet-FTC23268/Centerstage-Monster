package org.firstinspires.ftc.teamcode.Common.Subsystems;

import static org.firstinspires.ftc.teamcode.Common.Utility.Globals.DOWN_POS;
import static org.firstinspires.ftc.teamcode.Common.Utility.Globals.LIFT_ERROR_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Common.Utility.Globals.MAX_POS;
import static org.firstinspires.ftc.teamcode.Common.Utility.Globals.ROW10_POS;
import static org.firstinspires.ftc.teamcode.Common.Utility.Globals.ROW11_POS;
import static org.firstinspires.ftc.teamcode.Common.Utility.Globals.ROW1_POS;
import static org.firstinspires.ftc.teamcode.Common.Utility.Globals.ROW2_POS;
import static org.firstinspires.ftc.teamcode.Common.Utility.Globals.ROW3_POS;
import static org.firstinspires.ftc.teamcode.Common.Utility.Globals.ROW4_POS;
import static org.firstinspires.ftc.teamcode.Common.Utility.Globals.ROW5_POS;
import static org.firstinspires.ftc.teamcode.Common.Utility.Globals.ROW6_POS;
import static org.firstinspires.ftc.teamcode.Common.Utility.Globals.ROW7_POS;
import static org.firstinspires.ftc.teamcode.Common.Utility.Globals.ROW8_POS;
import static org.firstinspires.ftc.teamcode.Common.Utility.Globals.ROW9_POS;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.profile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.profile.ProfileState;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

@Config
public class DepositSubsystem extends SubsystemBase {

    private RobotHardware robot;

    public DepositState depositState = DepositState.INTAKE;

    private final ElapsedTime timer;

    public PIDController controller;

    public static double intermediate2Pos = 0.77;
    public static double intakePos = 0.72;
    public static double intermediatePos = 0.7;
    public static double readyPos = 0.55;
    public static double hangPos = 0.3;
    public static double deposit1Pos = 0.125;
    public static double deposit2Pos = 0.115;
    public static double deposit3Pos = 0.0675;
    public static double retractedPos = 0.16;
    public static double autonPos = 0;


    public static double leftOffset = 0;
    public static double rightOffset = 0;

    public double time = 0.0;

    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;

    public enum DepositState {
        INTAKE,
        INTERMEDIATE,
        INTERMEDIATE2,
        READY,
        HANG,
        DEPOSIT1,
        DEPOSIT2,
        DEPOSIT3,
        RETRACTED,
        AUTON,
    }

    public DepositSubsystem(RobotHardware robot) {

        this.robot = robot;
        this.controller = new PIDController(P, I, D);
        this.timer = new ElapsedTime();

    }

    public void update(DepositState state) {

        depositState = state;
        switch (state) {

            case INTAKE:
                robot.leftElbow.setPosition(intakePos + leftOffset);
                robot.rightElbow.setPosition(intakePos + rightOffset);
                break;
            case INTERMEDIATE:
                robot.leftElbow.setPosition(intermediatePos + leftOffset);
                robot.rightElbow.setPosition(intermediatePos + rightOffset);
                break;
            case INTERMEDIATE2:
                robot.leftElbow.setPosition(intermediate2Pos + leftOffset);
                robot.rightElbow.setPosition(intermediate2Pos + rightOffset);
                break;
            case READY:
                robot.leftElbow.setPosition(readyPos + leftOffset);
                robot.rightElbow.setPosition(readyPos + rightOffset);
                break;
            case HANG:
                robot.leftElbow.setPosition(hangPos + leftOffset);
                robot.rightElbow.setPosition(hangPos + rightOffset);
                break;
            case DEPOSIT1:
                robot.leftElbow.setPosition(deposit1Pos + leftOffset);
                robot.rightElbow.setPosition(deposit1Pos + rightOffset);
                break;
            case DEPOSIT2:
                robot.leftElbow.setPosition(deposit2Pos + leftOffset);
                robot.rightElbow.setPosition(deposit2Pos + rightOffset);
                break;
            case DEPOSIT3:
                robot.leftElbow.setPosition(deposit3Pos + leftOffset);
                robot.rightElbow.setPosition(deposit3Pos + rightOffset);
                break;
            case RETRACTED:
                robot.leftElbow.setPosition(retractedPos + leftOffset);
                robot.rightElbow.setPosition(retractedPos + rightOffset);
                break;
            case AUTON:
                robot.leftElbow.setPosition(autonPos + leftOffset);
                robot.rightElbow.setPosition(autonPos + rightOffset);
                break;

        }

    }

    public void resetTimer() {
        timer.reset();
    }

}
