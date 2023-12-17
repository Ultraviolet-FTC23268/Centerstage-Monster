package org.firstinspires.ftc.teamcode.Opmodes.weird.TestOps;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Commands.auton.PositionCommand;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.Vector2D;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

import org.firstinspires.ftc.teamcode.Common.Drivetrain.path.HermitePath;
//import org.firstinspires.ftc.teamcode.Common.Commands.auton.GVFCommand;

@Config
@Autonomous(name = "GVFTest")
@Disabled
public class GVFTest extends CommandOpMode {

    private ElapsedTime timer;

    private final RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private TwoWheelLocalizer localizer;

    private final HermitePath trajectory = new HermitePath()
            .addPose(0, 0, new Vector2D(0, 100))
            .addPose(0, 20, new Vector2D(0, 500))
            .addPose(20, 40, new Vector2D(500, 0))
            .addPose(40, 40, new Vector2D(100, 0))
            .flip()
            .construct();

    private double loopTime = 0.0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap, telemetry);
        robot.enabled = true;
        drivetrain = new SwerveDrivetrain(robot);
        localizer = new TwoWheelLocalizer(robot);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        robot.read(drivetrain, null);

        while (!isStarted()) {
            robot.read(drivetrain, null);
            drivetrain.frontLeftModule.setTargetRotation(0);
            drivetrain.frontRightModule.setTargetRotation(0);
            drivetrain.backRightModule.setTargetRotation(0);
            drivetrain.backLeftModule.setTargetRotation(0);
            drivetrain.updateModules();

            telemetry.addLine("auto in init");
            telemetry.update();
            robot.clearBulkCache();
            robot.write(drivetrain, null);
        }

        robot.startIMUThread(this);
        localizer.setPoseEstimate(new Pose2d(0, 0, 0));
        robot.reset();
        timer = new ElapsedTime();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        //new GVFCommand(drivetrain, localizer, trajectory),
                )
        );
    }

    @Override
    public void run() {

        robot.read(drivetrain, null);

        CommandScheduler.getInstance().run();

        robot.loop(null, drivetrain, null);
        localizer.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("xPos ", localizer.getPos().x);
        telemetry.addData("yPos ", localizer.getPos().y);
        telemetry.addData("hPos ", localizer.getPos().heading);
        loopTime = loop;
        telemetry.update();

        robot.write(drivetrain, null);
        robot.clearBulkCache();
    }
}
