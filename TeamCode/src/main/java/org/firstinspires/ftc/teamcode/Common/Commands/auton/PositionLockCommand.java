package org.firstinspires.ftc.teamcode.Common.Commands.auton;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

import java.util.function.BooleanSupplier;

public class PositionLockCommand extends CommandBase {
    public final double ALLOWED_TRANSLATIONAL_ERROR = 1;
    public final double ALLOWED_HEADING_ERROR = Math.toRadians(1.5);

    public final double PUSHED_TRANSLATIONAL_ERROR = 2;
    public final double PUSHED_HEADING_ERROR = Math.toRadians(3);

    public static double xP = 0.06;
    public static double xD = 0.03;
    public static double xF = 0;

    public static double yP = 0.06;
    public static double yD = 0.03;
    public static double yF = 0;

    public static double hP = 0.6;
    public static double hD = 0.3;
    public static double hF = 0;

    public static PIDFController xController = new PIDFController(xP, 0.0, xD, xF);
    public static PIDFController yController = new PIDFController(yP, 0.0, yD, yF);
    public static PIDFController hController = new PIDFController(hP, 0.0, hD, hF);
    public static double max_power = 1;
    public static double max_heading = 0.5;
    private static Pose targetPose;

    private final BooleanSupplier endSupplier;

    private ElapsedTime lockedTimer;
    private boolean reached;

    private final double v;

    private RobotHardware robot = RobotHardware.getInstance();

    public PositionLockCommand(BooleanSupplier end, double voltage) {
        this.v = voltage;
        this.endSupplier = end;
    }

    public static void setTargetPose(Pose target) {
        PositionLockCommand.targetPose = target;
    }

    public static Pose getTargetPose() {
        return PositionLockCommand.targetPose;
    }

    @Override
    public void initialize() {
        Globals.USE_WHEEL_FEEDFORWARD = true;
        lockedTimer = new ElapsedTime();
    }

    @Override
    public void execute() {
        if (targetPose.x == 0 && targetPose.y == 0 && targetPose.heading == 0) {
            robot.drivetrain.setLocked(false);
            return;
        }

        Pose powers = goToPosition(robot.localizer.getPos(), targetPose);

        if (!reached) lockedTimer.reset();

        robot.drivetrain.setLocked(lockedTimer.milliseconds() > 500);
        robot.drivetrain.set(reached ? new Pose() : powers);
    }

    @Override
    public boolean isFinished() {
        return endSupplier.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        robot.drivetrain.set(new Pose());
        Globals.USE_WHEEL_FEEDFORWARD = false;
    }

    private static Pose relDistanceToTarget(Pose robot, Pose target) {
        return target.subtract(robot);
    }

    public Pose goToPosition(Pose robotPose, Pose targetPose) {
        Pose deltaPose = relDistanceToTarget(robotPose, targetPose);

        if (Math.hypot(deltaPose.x, deltaPose.y) < ALLOWED_TRANSLATIONAL_ERROR && Math.abs(deltaPose.heading) < ALLOWED_HEADING_ERROR) {
            reached = true;
        }

        if (Math.hypot(deltaPose.x, deltaPose.y) > PUSHED_TRANSLATIONAL_ERROR || Math.abs(deltaPose.heading) > PUSHED_HEADING_ERROR) {
            reached = false;
        }

        Pose powers = new Pose(
                xController.calculate(0, deltaPose.x),
                yController.calculate(0, deltaPose.y),
                hController.calculate(0, deltaPose.heading)
        );
        double x_rotated = powers.x * Math.cos(robotPose.heading) - powers.y * Math.sin(robotPose.heading);
        double y_rotated = powers.x * Math.sin(robotPose.heading) + powers.y * Math.cos(robotPose.heading);
        double x_power = -x_rotated < -max_power ? -max_power :
                Math.min(-x_rotated, max_power);
        double y_power = -y_rotated < -max_power ? -max_power :
                Math.min(-y_rotated, max_power);
        double heading_power = MathUtils.clamp(powers.heading, -max_heading, max_heading);

        return new Pose(-y_power / v * 12, x_power / v * 12, -heading_power / v * 12);
    }
}
