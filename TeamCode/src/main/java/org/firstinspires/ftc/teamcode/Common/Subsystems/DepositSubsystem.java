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

    private RobotHardware robot = RobotHardware.getInstance();

    public DepositState depositState = DepositState.INTAKE;

    public DepositSubsystem.GateState gateState = DepositSubsystem.GateState.OPEN;

    private final ElapsedTime timer;

    public PIDController controller;

    public static double gateOpenPos = 0.4;
    public static double gateClosedPos = 0.55;

    public static double intakePos = 0.022;
    public static double intermediatePos = 0.06;
    public static double hangPos = 0.5;
    public static double depositPos = 0.87;
    public static double retractedPos = 0.01;


    public static double leftOffset = 0;
    public static double rightOffset = 0.01;

    public double time = 0.0;

    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;

    public enum DepositState {
        INTAKE,
        INTERMEDIATE,
        HANG,
        DEPOSIT,
        RETRACTED,
    }

    public enum GateState {
        CLOSED,
        OPEN
    }

    public DepositSubsystem() {

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
            case HANG:
                robot.leftElbow.setPosition(hangPos + leftOffset);
                robot.rightElbow.setPosition(hangPos + rightOffset);
                break;
            case DEPOSIT:
                robot.leftElbow.setPosition(depositPos + leftOffset);
                robot.rightElbow.setPosition(depositPos + rightOffset);
                break;
            case RETRACTED:
                robot.leftElbow.setPosition(retractedPos + leftOffset);
                robot.rightElbow.setPosition(retractedPos + rightOffset);
                break;

        }

    }

    public void update(GateState state) {

        gateState = state;
        switch (state) {

            case CLOSED:
                robot.gateServo.setPosition(gateClosedPos);
                break;
            case OPEN:
                robot.gateServo.setPosition(gateOpenPos);
                break;

        }

    }

    public void resetTimer() {
        timer.reset();
    }

}
