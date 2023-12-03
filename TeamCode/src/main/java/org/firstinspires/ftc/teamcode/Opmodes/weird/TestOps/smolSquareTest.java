package org.firstinspires.ftc.teamcode.Opmodes.weird.TestOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Common.Drivetrain.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Common.Commands.auton.PositionCommand;

@Photon
@Config
@Disabled
@TeleOp(name = "square but smol")
public class smolSquareTest extends CommandOpMode {
    private ElapsedTime timer;
    private double loopTime = 0;

    private int count = 0;


    private final RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    GamepadEx gamepadEx;

    TwoWheelLocalizer localizer;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Globals.AUTO = false;
        Globals.USE_WHEEL_FEEDFORWARD = true;
        Globals.USING_IMU = true;

        robot.init(hardwareMap, telemetry);
        drivetrain = new SwerveDrivetrain(robot);
        localizer = new TwoWheelLocalizer(robot);

        robot.enabled = true;

        drivetrain.frontLeftModule.setTargetRotation(0);
        drivetrain.frontRightModule.setTargetRotation(0);
        drivetrain.backRightModule.setTargetRotation(0);
        drivetrain.backLeftModule.setTargetRotation(0);

        while (!this.isStarted()) {
            drivetrain.read();
            drivetrain.updateModules();
            robot.clearBulkCache();

            telemetry.addData("t", drivetrain.frontLeftModule.getTargetRotation());
            telemetry.addData("c", drivetrain.frontLeftModule.getModuleRotation());
            telemetry.update();
        }
    }

    @Override
    public void run() {
        super.run();
        if (timer == null) {
            timer = new ElapsedTime();
            robot.reset();
            robot.startIMUThread(this);
            gamepadEx = new GamepadEx(gamepad1);
            localizer.setPoseEstimate(new Pose2d(0, 0, 0));
        }

        gamepadEx.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> schedule(new SequentialCommandGroup(
                        //1
                        /*
                        new PositionCommand(drivetrain, localizer, new Pose(24, 0, Math.toRadians(0)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, 0, Math.toRadians(90)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.toRadians(90)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.toRadians(180)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, Math.toRadians(180)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, Math.toRadians(270)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.toRadians(270)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.toRadians(0)), 0, 12.5),
                        //2
                        new PositionCommand(drivetrain, localizer, new Pose(24, 0, Math.toRadians(0)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, 0, Math.toRadians(90)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.toRadians(90)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.toRadians(180)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, Math.toRadians(180)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, Math.toRadians(270)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.toRadians(270)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.toRadians(0)), 0, 12.5),
                        //3
                        new PositionCommand(drivetrain, localizer, new Pose(24, 0, Math.toRadians(0)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, 0, Math.toRadians(90)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.toRadians(90)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.toRadians(180)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, Math.toRadians(180)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, Math.toRadians(270)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.toRadians(270)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.toRadians(0)), 0, 12.5),
                        //4
                        new PositionCommand(drivetrain, localizer, new Pose(24, 0, Math.toRadians(0)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, 0, Math.toRadians(90)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.toRadians(90)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.toRadians(180)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, Math.toRadians(180)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, Math.toRadians(270)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.toRadians(270)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.toRadians(0)), 0, 12.5),
                        //5
                        new PositionCommand(drivetrain, localizer, new Pose(24, 0, Math.toRadians(0)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, 0, Math.toRadians(90)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.toRadians(90)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.toRadians(180)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, Math.toRadians(180)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, Math.toRadians(270)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.toRadians(270)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.toRadians(0)), 0, 12.5),
                        //6
                        new PositionCommand(drivetrain, localizer, new Pose(24, 0, Math.toRadians(0)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, 0, Math.toRadians(90)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.toRadians(90)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.toRadians(180)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, Math.toRadians(180)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, Math.toRadians(270)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.toRadians(270)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.toRadians(0)), 0, 12.5),
                        //7
                        new PositionCommand(drivetrain, localizer, new Pose(24, 0, Math.toRadians(0)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, 0, Math.toRadians(90)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.toRadians(90)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.toRadians(180)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, Math.toRadians(180)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, Math.toRadians(270)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.toRadians(270)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.toRadians(0)), 0, 12.5),
                        //8
                        new PositionCommand(drivetrain, localizer, new Pose(24, 0, Math.toRadians(0)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, 0, Math.toRadians(90)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.toRadians(90)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.toRadians(180)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, Math.toRadians(180)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, Math.toRadians(270)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.toRadians(270)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.toRadians(0)), 0, 12.5),
                        //9
                        new PositionCommand(drivetrain, localizer, new Pose(24, 0, Math.toRadians(0)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, 0, Math.toRadians(90)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.toRadians(90)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.toRadians(180)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, Math.toRadians(180)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, Math.toRadians(270)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.toRadians(270)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.toRadians(0)), 0, 12.5),
                        //10
                        new PositionCommand(drivetrain, localizer, new Pose(24, 0, Math.toRadians(0)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, 0, Math.toRadians(90)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.toRadians(90)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(24, -24, Math.toRadians(180)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, Math.toRadians(180)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, -24, Math.toRadians(270)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.toRadians(270)), 0, 12.5),
                        new PositionCommand(drivetrain, localizer, new Pose(0, 0, Math.toRadians(0)), 0, 12.5)*/
                )));

        gamepadEx.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> count+=10);


        drivetrain.read();
        drivetrain.updateModules();
        drivetrain.write();
        localizer.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("x", localizer.getPos().x);
        telemetry.addData("y", localizer.getPos().y);
        telemetry.addData("h", localizer.getPos().heading);
        telemetry.addData("xP", PositionCommand.xP);
        telemetry.addData("xD", PositionCommand.xD);
        telemetry.addData("Ct:", count);
        //telemetry.addData("t", drivetrain.frontLeftModule.getTargetRotation());
        //telemetry.addData("c", drivetrain.frontLeftModule.getModuleRotation());
        loopTime = loop;
        telemetry.update();

        robot.clearBulkCache();
    }
}
