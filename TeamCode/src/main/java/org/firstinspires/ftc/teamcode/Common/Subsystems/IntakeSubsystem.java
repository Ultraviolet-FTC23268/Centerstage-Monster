package org.firstinspires.ftc.teamcode.Common.Subsystems;

import org.firstinspires.ftc.teamcode.Common.Utility.Globals;

import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@Config
public class IntakeSubsystem {

    private RobotHardware robot = RobotHardware.getInstance();

    public IntakeState intakeState = IntakeState.OFF;

    public static double intakeInSpeed = -1;
    public static double intakeOutSpeed = 1;

    public enum IntakeState {
        OFF,
        INWARDS,
        OUTWARDS
    }

    public IntakeSubsystem() {

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
            case OUTWARDS:
                robot.intakeMotor.setPower(intakeOutSpeed);
                //robot.LEDcontroller.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_FAST);
                break;

        }

    }


}
