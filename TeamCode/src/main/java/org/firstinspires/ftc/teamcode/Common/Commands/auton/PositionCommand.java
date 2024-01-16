package org.firstinspires.ftc.teamcode.Common.Commands.auton;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Drivetrain.swerve.Drivetrain;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.localizer.Localizer;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

@Config
public class PositionCommand extends CommandBase {
    public static double ALLOWED_TRANSLATIONAL_ERROR = 0.25;
    public static double ALLOWED_HEADING_ERROR = Math.toRadians(1);

    public static double xP = -0.0365;
    public static double xD = 0.005;
    public static double xF = 0;

    public static double yP = 0.0365;
    public static double yD = 0.005;
    public static double yF = 0;

    public static double hP = 0.575;
    public static double hD = 0.1;
    public static double hF = 0;

    public static PIDFController xController;
    public static PIDFController yController;
    public static PIDFController hController;
    public static double max_power = 1;
    public static double max_heading = 0.5;

    Drivetrain drivetrain;
    Localizer localizer;
    Pose targetPose;
    ElapsedTime deadTimer;

    private final double ms;
    private final double delay;
    private ElapsedTime delayTimer;

    private final double v;

    private RobotHardware robot = RobotHardware.getInstance();

    private ElapsedTime timer;
    private ElapsedTime stable;

    public PositionCommand(Drivetrain drivetrain, Localizer localizer, Pose targetPose, double delay, double dead, double voltage) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.targetPose = targetPose;
        this.ms = dead;
        this.delay = delay;
        this.v = voltage;

        xController = new PIDFController(xP, 0.0, xD, 0);
        yController = new PIDFController(yP, 0.0, yD, 0);
        hController = new PIDFController(hP, 0.0, hD, 0);
    }

    public PositionCommand(Drivetrain drivetrain, Localizer localizer, Pose targetPose, double delay, double voltage) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.targetPose = targetPose;
        this.ms = Integer.MAX_VALUE;
        this.delay = delay;
        this.v = voltage;

        xController = new PIDFController(xP, 0.0, xD, 0);
        yController = new PIDFController(yP, 0.0, yD, 0);
        hController = new PIDFController(hP, 0.0, hD, 0);
    }

    @Override
    public void initialize(){
        Globals.USE_WHEEL_FEEDFORWARD = true;
    }

    @Override
    public void execute() {
        if (deadTimer == null) {
            deadTimer = new ElapsedTime();
        }

        Pose powers = goToPosition(localizer.getPos(), targetPose);
        drivetrain.set(powers);
    }

    @Override
    public boolean isFinished() {
        Pose error = targetPose.subtract(localizer.getPos());

        boolean reached = ((Math.hypot(error.x, error.y) < ALLOWED_TRANSLATIONAL_ERROR) && (Math.abs(error.heading) < ALLOWED_HEADING_ERROR));

        if (reached && delayTimer == null) {
            delayTimer = new ElapsedTime();
        }
        if (!reached && delayTimer != null) {
            delayTimer.reset();
        }

        boolean delayed = delayTimer != null && delayTimer.milliseconds() > delay;
        return (deadTimer.milliseconds() > ms) || delayed;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.set(new Pose());
        Globals.USE_WHEEL_FEEDFORWARD = false;
    }

    private static Pose relDistanceToTarget(Pose robot, Pose target) {
        return target.subtract(robot);
    }

    public Pose goToPosition(Pose robotPose, Pose targetPose) {
        Pose deltaPose = relDistanceToTarget(robotPose, targetPose);
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

        if(Math.abs(x_power) < 0.01) x_power = 0;
        if(Math.abs(y_power) < 0.01) y_power = 0;

        return new Pose(-y_power / v * 12, x_power / v * 12, -heading_power / v * 12);
    }
}
