package org.firstinspires.ftc.teamcode.Funny.commandbase.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Funny.Drivetrain.geometry.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.Funny.Drivetrain.geometry.Constraints;
import org.firstinspires.ftc.teamcode.Funny.Drivetrain.geometry.State;
import org.firstinspires.ftc.teamcode.Funny.Utility.RobotHardware;

import static org.firstinspires.ftc.teamcode.Funny.Utility.Globals.*;

public class LiftSubsystem {

    private RobotHardware robot;

    //public LiftState liftState = LiftState.OFF;

    private final ElapsedTime timer;
    private AsymmetricMotionProfile liftProfile;
    public State liftMotionState;
    public LiftStateReel liftStateReel = LiftStateReel.DOWN;

    public PIDController controller;

    public int leftLiftPos;
    public int rightLiftPos;
    public double power = 0.0;
    private int targetPosition = 0;
    public static double liftRaiseSpeed = 1;
    public static double liftLowerSpeed = -1;
    private boolean withinTolerance = false;
    public boolean isReady = false;

    public double time = 0.0;
    public boolean resetting = false;

    public static double P = 0;
    public static double I = 0;
    public static double D = 0;

    /*
    public enum LiftState {
        OFF,
        UPWARDS,
        DOWNWARDS
    }*/

    public enum LiftStateReel {
        DOWN,
        ROW1,
        ROW2,
        ROW3,
        ROW4,
        ROW5,
        ROW6,
        ROW7,
        ROW8,
        ROW9,
        ROW10,
        ROW11
    }

    public LiftSubsystem(RobotHardware robot) {

        this.robot = robot;
        this.liftProfile = new AsymmetricMotionProfile(0, 1, new Constraints(0, 0, 0));
        this.controller = new PIDController(P, I, D);
        this.timer = new ElapsedTime();

    }


  /*  public void update(LiftState state) {

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

    }*/

    public void update(LiftStateReel state) {

        liftStateReel = state;
        switch (state) {

            case DOWN:
                setTargetPos(DOWN_POS);
                break;
            case ROW1:
                setTargetPos(ROW1_POS);
                break;
            case ROW2:
                setTargetPos(ROW2_POS);
                break;
            case ROW3:
                setTargetPos(ROW3_POS);
                break;
            case ROW4:
                setTargetPos(ROW4_POS);
                break;
            case ROW5:
                setTargetPos(ROW5_POS);
                break;
            case ROW6:
                setTargetPos(ROW6_POS);
                break;
            case ROW7:
                setTargetPos(ROW7_POS);
                break;
            case ROW8:
                setTargetPos(ROW8_POS);
                break;
            case ROW9:
                setTargetPos(ROW9_POS);
                break;
            case ROW10:
                setTargetPos(ROW10_POS);
                break;
            case ROW11:
                setTargetPos(ROW11_POS);
                break;

        }

    }

    public void loop() {
        this.controller.setPID(P, I, D);

        liftMotionState = liftProfile.calculate(timer.time());
        if (liftMotionState.v != 0) {
            setTargetPos((int) liftMotionState.x);
            time = timer.time();
        }

        //withinTolerance = Math.abs(getLeftPos() - getTargetPos()) < LIFT_ERROR_TOLERANCE;

        //power = Range.clip(((-controller.calculate(leftLiftPos, targetPosition) + (F * Math.signum(targetPosition - leftLiftPos))) / robot.getVoltage() * 14), -1, 1);
        //power = Range.clip(((-controller.calculate(rightLiftPos, targetPosition) + (F * Math.signum(targetPosition - rightLiftPos))) / robot.getVoltage() * 14), -1, 1);

        if (resetting) {
            power = 0.4; //retracting
        }
    }
    public void read() {
        try {
            leftLiftPos = robot.leftArmEncoder.getPosition();
            rightLiftPos = robot.rightArmEncoder.getPosition();

        } catch (Exception e) {
            leftLiftPos = 0;
            rightLiftPos = 0;
        }
    }

    public double getLeftPos() {
        return leftLiftPos;
    }

    public double getRightPos() {
        return rightLiftPos;
    }

    public void setTargetPos(int targetPosition) {
        this.targetPosition = targetPosition;
    }

    public int getTargetPos() {
        return targetPosition;
    }

    public int getStatePos(LiftStateReel state) {
        switch (state) {
            case DOWN:
                return DOWN_POS;
            case ROW1:
                return ROW1_POS;
            case ROW2:
                return ROW2_POS;
            case ROW3:
                return ROW3_POS;
            case ROW4:
                return ROW4_POS;
            case ROW5:
                return ROW5_POS;
            case ROW6:
                return ROW6_POS;
            case ROW7:
                return ROW7_POS;
            case ROW8:
                return ROW8_POS;
            case ROW9:
                return ROW9_POS;
            case ROW10:
                return ROW10_POS;
            case ROW11:
                return ROW11_POS;
        }
        return 0;
    }
    public double getPower() {
        return power;
    }
    public void setReady(boolean ready) {
        isReady = ready;
    }

    public boolean isReady() {
        return isReady;
    }
    public boolean isWithinTolerance() {
        return withinTolerance;
    }

    public void resetTimer() {
        timer.reset();
    }

    public void newProfile(double targetPos, Constraints constraints) {
        //constraints.convert(LIFT_TICKS_PER_INCH);
        this.liftProfile = new AsymmetricMotionProfile(getLeftPos(), targetPos, constraints);
        resetTimer();
    }


}
