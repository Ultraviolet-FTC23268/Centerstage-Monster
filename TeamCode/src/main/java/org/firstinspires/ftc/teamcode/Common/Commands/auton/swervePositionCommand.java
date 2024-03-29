package org.firstinspires.ftc.teamcode.Common.Commands.auton;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Drivetrain.swerve.Drivetrain;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.localizer.Localizer;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

@Config
public class swervePositionCommand extends CommandBase {
    Localizer localizer;
    Drivetrain drivetrain;
    public Pose targetPose;
    private final double v;

    public static double max_power = 1;
    public static double max_heading = 1;

    public static double xP = -0.0365; //0.0385
    public static double xD = 0.005; //0.005

    public static double yP = 0.0365; //0.0385
    public static double yD = 0.005; //0.005

    public static double hP = 0.525; //0.75
    public static double hD = 0; //0.02

    //public static PIDFController xController;
    //public static PIDFController yController;
    //public static PIDFController hController;

    public static PIDFController xController = new PIDFController(xP, 0.0, xD, 0);
    public static PIDFController yController = new PIDFController(yP, 0.0, yD, 0);
    public static PIDFController hController = new PIDFController(hP, 0.0, hD, 0);

    public static double ALLOWED_TRANSLATIONAL_ERROR = 1.5;
    public static double ALLOWED_HEADING_ERROR = 0.03;

    private RobotHardware robot = RobotHardware.getInstance();

    private ElapsedTime timer;
    private ElapsedTime stable;


    public swervePositionCommand(Drivetrain drivetrain, Localizer localizer, Pose targetPose, double voltage) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.targetPose = targetPose;
        this.v = voltage;

        /*xController = new PIDFController(xP, 0.0, xD, 0);
        yController = new PIDFController(yP, 0.0, yD, 0);
        hController = new PIDFController(hP, 0.0, hD, 0);*/
    }

    @Override
    public void execute() {
        if (timer == null) timer = new ElapsedTime();
        if (stable == null) stable = new ElapsedTime();

        Pose robotPose = localizer.getPos();

        Pose powers = getPower(robotPose);
        drivetrain.set(powers);
    }

    @Override
    public boolean isFinished() {
        Pose robotPose = localizer.getPos();
        Pose delta = targetPose.subtract(robotPose);

        if (delta.toVec2D().magnitude() > ALLOWED_TRANSLATIONAL_ERROR
                || Math.abs(delta.heading) > ALLOWED_HEADING_ERROR) {
            stable.reset();
        }

        return timer.milliseconds() > 5000 || stable.milliseconds() > 250;
    }

    public Pose getPower(Pose robotPose) {
        Pose deltaPose = targetPose.subtract(robotPose);
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

    @Override
    public void end(boolean interrupted) {
        drivetrain.set(new Pose());
    }
}