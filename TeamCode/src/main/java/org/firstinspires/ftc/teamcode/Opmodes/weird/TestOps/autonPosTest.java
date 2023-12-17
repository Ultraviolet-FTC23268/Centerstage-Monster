package org.firstinspires.ftc.teamcode.Opmodes.weird.TestOps;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Commands.auton.KbPositionCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.auton.PositionCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.auton.swervePositionCommand;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.swerve.Drivetrain;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

@Config
@Autonomous(name = "PosAutoTest")
@Disabled
public class autonPosTest extends CommandOpMode {

    private ElapsedTime timer;

    private final RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private TwoWheelLocalizer localizer;

    private double loopTime = 0.0;

    public static double posX = 0;
    public static double posY = 0;
    public static double posH = 0;

    Pose testPos = new Pose();

    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap, telemetry);
        robot.enabled = true;
        drivetrain = new SwerveDrivetrain(robot);
        localizer = new TwoWheelLocalizer(robot);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        localizer.setPoseEstimate(new Pose2d(0, 0, 0));
        testPos = new Pose(posX, posY, posH);

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
        robot.reset();
        timer = new ElapsedTime();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        //new PositionCommand(drivetrain, localizer, testPos)
                        new swervePositionCommand(drivetrain, localizer, testPos, 12.5)
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
