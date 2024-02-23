package org.firstinspires.ftc.teamcode.Opmodes.weird.TestOps;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Commands.auton.swervePositionCommand;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

@Config
@Autonomous(name = "AutonPosTest")
//@Disabled
public class autonPosTest extends CommandOpMode {

    private ElapsedTime timer;

    private final RobotHardware robot = RobotHardware.getInstance();

    private double loopTime = 0.0;

    public static double posX = 0;
    public static double posY = 0;
    public static double posH = 0;

    public static double posX2 = 0;
    public static double posY2 = 0;
    public static double posH2 = 0;

    public static int dead = 15000;
    public static int dead2 = 15000;

    Pose testPos = new Pose();
    Pose testPos2 = new Pose();

    @Override
    public void initialize() {

        Globals.USE_WHEEL_FEEDFORWARD = true;
        Globals.AUTO = true;
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap, telemetry);
        robot.enabled = true;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

        robot.localizer.setPoseEstimate(new Pose2d());
        testPos = new Pose(posX, posY, posH);
        testPos2 = new Pose(posX2, posY2, posH2);

        robot.read();

        while (!isStarted()) {
            robot.read();
            robot.drivetrain.frontLeftModule.setTargetRotation(0);
            robot.drivetrain.frontRightModule.setTargetRotation(0);
            robot.drivetrain.backRightModule.setTargetRotation(0);
            robot.drivetrain.backLeftModule.setTargetRotation(0);
            robot.drivetrain.updateModules();

            telemetry.addLine("auto in init");
            telemetry.update();
            robot.clearBulkCache();
            robot.write();
        }

        robot.localizer.setPoseEstimate(new Pose2d(0, 0, 0));

        robot.startIMUThread(this);
        robot.reset();
        timer = new ElapsedTime();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new swervePositionCommand(testPos, dead),
                        new WaitCommand(1000),
                        new swervePositionCommand(testPos2, dead2),
                        new WaitCommand(1000)
                )
        );
    }

    @Override
    public void run() {

        robot.clearBulkCache();
        robot.read();
        robot.loop(null);
        robot.write();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("xPos ", robot.localizer.getPos().x);
        telemetry.addData("yPos ", robot.localizer.getPos().y);
        telemetry.addData("hPos ", robot.localizer.getPos().heading);
        loopTime = loop;
        telemetry.update();

        CommandScheduler.getInstance().run();

    }
}
