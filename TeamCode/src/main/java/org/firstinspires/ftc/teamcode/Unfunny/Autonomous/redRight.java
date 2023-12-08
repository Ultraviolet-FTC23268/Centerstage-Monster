package org.firstinspires.ftc.teamcode.Unfunny.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Funny.commandbase.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Funny.Drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.Funny.Drivetrain.geometry.Pose;
import org.firstinspires.ftc.teamcode.Funny.Drivetrain.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.Funny.Utility.Globals;
import org.firstinspires.ftc.teamcode.Funny.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Funny.commandbase.auton.PositionCommand;
import org.firstinspires.ftc.teamcode.Funny.commandbase.bobot.IntakeCommand;



import java.util.function.DoubleSupplier;

@Autonomous(name = "Red Backdrop ⮕")
public class redRight extends LinearOpMode{

    private RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private IntakeSubsystem intake;
    private TwoWheelLocalizer localizer;
    private ElapsedTime timer;

    private double loopTime;
    private double endtime;

    @Override
    public void runOpMode() /*throws InterruptedException*/ {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CommandScheduler.getInstance().reset();
        Globals.AUTO = true;
        Globals.USING_IMU = true;
        Globals.SIDE = Globals.Side.LEFT;

        robot.init(hardwareMap, telemetry);
        drivetrain = new SwerveDrivetrain(robot);
        intake = new IntakeSubsystem(robot);
        localizer = new TwoWheelLocalizer(robot);

        robot.enabled = true;

        //robot.LEDcontroller.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);

        telemetry.addLine("RAAAAHHHHHHH");

        while (!opModeIsActive()) {

            robot.read(drivetrain);
            drivetrain.frontLeftModule.setTargetRotation(0);
            drivetrain.frontRightModule.setTargetRotation(0);
            drivetrain.backLeftModule.setTargetRotation(0);
            drivetrain.backRightModule.setTargetRotation(0);
            drivetrain.updateModules();

            telemetry.addData("x", localizer.getPos().x);
            telemetry.addData("y", localizer.getPos().y);
            telemetry.update();

            robot.clearBulkCache();
            robot.write(drivetrain);

        }

        robot.startIMUThread(this);
        localizer.setPoseEstimate(new Pose2d(0, 0, -Math.PI/2));
        robot.reset();
        timer = new ElapsedTime();

        DoubleSupplier time_left = () -> 30 - timer.seconds();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        //Deposit Preload
                        new PositionCommand(drivetrain, localizer, new Pose(8, 36, -Math.PI/2), 0, robot.getVoltage()),
                        new IntakeCommand(intake, IntakeSubsystem.IntakeState.OUTWARDS)
                                .alongWith(new WaitCommand(500)),
                        new IntakeCommand(intake, IntakeSubsystem.IntakeState.OFF),
                        new PositionCommand(drivetrain, localizer, new Pose(60, 10, Math.PI/2), 0, robot.getVoltage()),
                        new PositionCommand(drivetrain, localizer, new Pose(48.5, -55, Math.PI/2), 0, robot.getVoltage()),
                        //Pickup Stack
                        new PositionCommand(drivetrain, localizer, new Pose(48.5, -68, Math.PI/2), 0, robot.getVoltage()),
                        new IntakeCommand(intake, IntakeSubsystem.IntakeState.INWARDS)
                                .alongWith(new WaitCommand(1250)),
                        //new IntakeCommand(intake, IntakeSubsystem.IntakeState.OFF),
                        //Deposit Stack
                        new PositionCommand(drivetrain, localizer, new Pose(52.5, 37.5, -Math.PI/2-Math.toRadians(27.5)), 0, robot.getVoltage()),
                        new IntakeCommand(intake, IntakeSubsystem.IntakeState.OUTWARDS)
                                .alongWith(new WaitCommand(1750)),
                        new IntakeCommand(intake, IntakeSubsystem.IntakeState.OFF),
                        new PositionCommand(drivetrain, localizer, new Pose(48.5, -55, Math.PI/2), 0, robot.getVoltage()),
                        new PositionCommand(drivetrain, localizer, new Pose(34, -66.25, Math.PI/2), 0, robot.getVoltage()),
                        //Pickup Stack 2
                        new PositionCommand(drivetrain, localizer, new Pose(34, -66.5, Math.PI/2), 0, robot.getVoltage()),
                        new IntakeCommand(intake, IntakeSubsystem.IntakeState.INWARDS)
                                .alongWith(new WaitCommand(1250)),
                        //new IntakeCommand(intake, IntakeSubsystem.IntakeState.OFF),
                        new PositionCommand(drivetrain, localizer, new Pose(52, -55, Math.PI/2), 0, robot.getVoltage()),
                        //Deposit Stack 2
                        new PositionCommand(drivetrain, localizer, new Pose(52.5, 37.5, -Math.PI/2-Math.toRadians(27.5)), 0, robot.getVoltage()),
                        new IntakeCommand(intake, IntakeSubsystem.IntakeState.OUTWARDS)
                                .alongWith(new WaitCommand(1750)),
                        new IntakeCommand(intake, IntakeSubsystem.IntakeState.OFF),

                        new InstantCommand(() -> endtime = timer.seconds())
                )
        );

        waitForStart();

        while (opModeIsActive()) {

            robot.read(drivetrain);

            CommandScheduler.getInstance().run();
            robot.loop(null, drivetrain);
            localizer.periodic();

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addData("endtime", endtime);
            telemetry.addData("x", localizer.getPos().x);
            telemetry.addData("y", localizer.getPos().y);
            telemetry.addData("h", localizer.getPos().heading);
            loopTime = loop;
            telemetry.update();
            robot.write(drivetrain);
            robot.clearBulkCache();
        }

    }

    public static int getModifier(){
        return Globals.SIDE == Globals.Side.LEFT ? 1 : -1;
    }
}
