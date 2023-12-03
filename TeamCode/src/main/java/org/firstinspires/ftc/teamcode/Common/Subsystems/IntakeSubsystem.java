package org.firstinspires.ftc.teamcode.Common.Subsystems;

import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


public class IntakeSubsystem {

    private RobotHardware robot;

    public IntakeState intakeState = IntakeState.OFF;

    public PIDController controller;

    public static double intakeIdleSpeed = -0.6;
    public static double intakeInSpeed = -1;
    public static double intakeOutSpeed = 1;

    public static double P = 0;
    public static double I = 0;
    public static double D = 0;

    public enum IntakeState {
        OFF,
        INWARDS,
        IDLE,
        OUTWARDS
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
                robot.LEDcontroller.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);
                break;
            case INWARDS:
                robot.intakeMotor.setPower(intakeInSpeed);
                robot.LEDcontroller.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
                break;
            case IDLE:
                robot.intakeMotor.setPower(intakeIdleSpeed);
                break;
            case OUTWARDS:
                robot.intakeMotor.setPower(intakeOutSpeed);
                robot.LEDcontroller.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_FAST);
                break;

        }

    }

    public void loop2() {}


}
