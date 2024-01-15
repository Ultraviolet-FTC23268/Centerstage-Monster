package org.firstinspires.ftc.teamcode.Common.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.profile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.profile.ProfileState;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

import static org.firstinspires.ftc.teamcode.Common.Utility.Globals.*;

@Config
public class LiftSubsystem extends SubsystemBase {

    private RobotHardware robot;

    private final ElapsedTime timer;
    private AsymmetricMotionProfile liftProfile;
    public ProfileState liftMotionState;
    public LiftStateReel liftStateReel = LiftStateReel.DOWN;

    public PIDController controller;

    public int leftLiftPos;
    public int rightLiftPos;
    public double power = 0.0;

    public static int rowPos = 1;
    public static int targetPosition = 0;
    public static double liftRaiseSpeed = 1;
    public static double liftLowerSpeed = -1;
    private boolean withinTolerance = false;
    public boolean isReady = false;

    public double time = 0.0;
    public static boolean isUp = false;

    public static double P = 0.0075;
    public static double I = 0;
    public static double D = 0.0003;
    public static double F = 0;

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
        ROW11,
        MAX
    }

    public LiftSubsystem(RobotHardware robot) {

        this.robot = robot;
        this.liftProfile = new AsymmetricMotionProfile(0, 1, new ProfileConstraints(0, 0, 0));
        this.controller = new PIDController(P, I, D);
        this.timer = new ElapsedTime();

    }

    public void update(LiftStateReel state) {
        liftStateReel = state;

        if(state == LiftStateReel.DOWN)
            isUp = false;
        else
            isUp = true;

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
            case MAX:
                setTargetPos(MAX_POS);
                break;

        }

    }

    public void updateRowPos(int row) {

        switch (row) {
            case 1:
                this.update(LiftStateReel.ROW1);
                break;
            case 2:
                this.update(LiftStateReel.ROW2);
                break;
            case 3:
                this.update(LiftStateReel.ROW3);
                break;
            case 4:
                this.update(LiftStateReel.ROW4);
                break;
            case 5:
                this.update(LiftStateReel.ROW5);
                break;
            case 6:
                this.update(LiftStateReel.ROW6);
                break;
            case 7:
                this.update(LiftStateReel.ROW7);
                break;
            case 8:
                this.update(LiftStateReel.ROW8);
                break;
            case 9:
                this.update(LiftStateReel.ROW9);
                break;
            case 10:
                this.update(LiftStateReel.ROW10);
                break;
            case 11:
                this.update(LiftStateReel.ROW11);
                break;
        }

    }

    public void changeRow(int amount) {

        if(isUp)
            rowPos += amount;

        if(rowPos > 11)
            rowPos = 11;
        if(rowPos < 1)
            rowPos = 1;

        updateRowPos(rowPos);

    }

    public void loop() {
        this.controller.setPID(P, I, D);

        liftMotionState = liftProfile.calculate(timer.time());
        if (liftMotionState.v != 0) {
            setTargetPos((int) liftMotionState.x);
            time = timer.time();
        }

        if(rowPos > 11)
            rowPos=11;
        if(rowPos < 1)
            rowPos=1;

        withinTolerance = Math.abs(getRightPos() - getTargetPos()) < LIFT_ERROR_TOLERANCE;

        power = Range.clip(((-controller.calculate(rightLiftPos, targetPosition) + (F * Math.signum(targetPosition - rightLiftPos))) / robot.getVoltage() * 14), -1, 1);

        if (isWithinTolerance()) {
            power = 0; //Turn off lift
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

    public void write() {
        if (robot.enabled) {
            try {
                robot.armLeftMotor.set(power);
                robot.armRightMotor.set(power);
            } catch (Exception e) {
            }
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
            case MAX:
                return MAX_POS;
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

    public void newProfile(double targetPos, ProfileConstraints constraints) {
        //constraints.convert(LIFT_TICKS_PER_INCH);
        this.liftProfile = new AsymmetricMotionProfile(getLeftPos(), targetPos, constraints);
        resetTimer();
    }


}
