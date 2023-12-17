package org.firstinspires.ftc.teamcode.Common.Subsystems;

import static org.firstinspires.ftc.teamcode.Common.Utility.Globals.DOWN_POS;
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

import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@Config
public class IntakeSubsystem {

    private RobotHardware robot;

    public IntakeState intakeState = IntakeState.OFF;
    public GateState gateState = GateState.OPEN;

    public PIDController controller;

    public static double intakeMaxSpeed = -1;
    public static double intakeInSpeed = -.66;
    public static double intakeOutSpeed = 1;

    public static double intakeOutSpeedAuton = .2;

    public static double gateOpenPos = 0;

    public static double gateClosedPos = 0;

    public static double P = 0;
    public static double I = 0;
    public static double D = 0;

    public enum IntakeState {
        OFF,
        INWARDS,
        MAX,
        OUTWARDS,

        AUTON_OUTWARDS,
    }

    public enum GateState {
        CLOSED,
        OPEN,
    }

    public IntakeSubsystem(RobotHardware robot) {

        this.robot = robot;
        this.controller = new PIDController(P, I, D);

    }


    public void update(IntakeState state) {

        intakeState = state;
        switch (state) {

            case OFF:
                robot.intakeMotor.setPower(0);
                //robot.LEDcontroller.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);
                break;
            case INWARDS:
                robot.intakeMotor.setPower(intakeInSpeed);
                //robot.LEDcontroller.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
                break;
            case MAX:
                robot.intakeMotor.setPower(intakeMaxSpeed);
                break;
            case OUTWARDS:
                robot.intakeMotor.setPower(intakeOutSpeed);
                //robot.LEDcontroller.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_FAST);
                break;
            case AUTON_OUTWARDS:
                robot.intakeMotor.setPower(intakeOutSpeedAuton);
                //robot.LEDcontroller.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_FAST);
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


}
