package org.firstinspires.ftc.teamcode.Funny.commandbase.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.Funny.Utility.RobotHardware;


public class LiftSubsystem {

    private RobotHardware robot;

    public LiftState liftState = LiftState.OFF;

    public PIDController controller;

    public static double liftRaiseSpeed = 1;
    public static double liftLowerSpeed = -1;

    public static double P = 0;
    public static double I = 0;
    public static double D = 0;

    public enum LiftState {
        OFF,
        UPWARDS,
        DOWNWARDS
    }

    public LiftSubsystem(RobotHardware robot) {

        this.robot = robot;
        this.controller = new PIDController(P, I, D);

    }


    public void update(LiftState state) {

        liftState = state;
        switch (state) {

            case OFF:
                robot.armLeftMotor.setPower(0);
                robot.armRightMotor.setPower(0);
                break;
            case UPWARDS:
                robot.armLeftMotor.setPower(liftRaiseSpeed);
                robot.armRightMotor.setPower(liftRaiseSpeed);
                break;
            case DOWNWARDS:
                robot.armLeftMotor.setPower(liftLowerSpeed);
                robot.armRightMotor.setPower(liftLowerSpeed);
                break;

        }

    }

    public void loop2() {}


}
