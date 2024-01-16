package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.Preload;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.GateCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.auton.CancelableResetArmCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.auton.swervePositionCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.teleop.MoveArmCommand;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Other.Pipelines.PropPipeline;
import org.firstinspires.ftc.teamcode.Other.Side;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "\uD83D\uDFE5 Close Preload Auto")
public class preloadRedCloseAutonFIX extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private TwoWheelLocalizer localizer;
    private DepositSubsystem deposit;
    private IntakeSubsystem intake;

    private LiftSubsystem lift;

    private PropPipeline propPipeline;
    private VisionPortal portal;


    private double loopTime = 0.0;

    public static int scoreDelay = 500;

    public static int intakeScoreLength = 500;

    public static double preYellowPosX = -28;
    public static double preYellowPosY= 21;
    public static double preYellowPosH = Math.PI;

    public static double yellowPosX = -35.5;
    public static double yellowPosY= 21;
    public static double yellowPosH = Math.PI;

    public static double purplePosX = 6;
    public static double purplePosY= 32;
    public static double purplePosH = 0;

    public static double parkPosX = -36;
    public static double parkPosY= 5;
    public static double parkPosH = 0;

    public static int preYellowOverride = 2000;
    public static int yellowOverride = 1000;
    public static int prePurpleOverride = 1500;
    public static int purpleOverride = 2500;
    public static int parkOverride = 2500;

    public static int bucketHeightOffset = 0;

    public static double hCeOffset = 0;
    public static double hLeOffset = -5;
    public static double hRiOffset = -5;

    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();

        Globals.COLOR = Side.BLUE;
        Globals.SIDE = Side.LEFT;
        Globals.USE_WHEEL_FEEDFORWARD = true;

        robot.init(hardwareMap, telemetry);
        robot.enabled = true;
        drivetrain = new SwerveDrivetrain(robot);
        localizer = new TwoWheelLocalizer(robot);
        lift = new LiftSubsystem(robot);
        deposit = new DepositSubsystem(robot);
        intake = new IntakeSubsystem(robot);

        localizer.setPoseEstimate(new Pose2d(0, 0, 0));

        propPipeline = new PropPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1280, 720))
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(propPipeline)
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        deposit.update(DepositSubsystem.DepositState.INTAKE);
        deposit.update(DepositSubsystem.GateState.CLOSED);

        robot.read(drivetrain, null);
        while (!isStarted()) {
            robot.read(drivetrain, null);
            drivetrain.frontLeftModule.setTargetRotation(0);
            drivetrain.frontRightModule.setTargetRotation(0);
            drivetrain.backRightModule.setTargetRotation(0);
            drivetrain.backLeftModule.setTargetRotation(0);
            drivetrain.updateModules();

            telemetry.addLine("auto in init");
            telemetry.addData("POS", propPipeline.getLocation());
            telemetry.update();

            robot.clearBulkCache();
            robot.write(drivetrain, null);
        }

        localizer.setPoseEstimate(new Pose2d(0, 0, 0));

        Side side = propPipeline.getLocation();
        portal.close();

        Pose preYellowScorePos = new Pose();
        Pose yellowScorePos = new Pose();
        Pose prePurplePos = new Pose();
        Pose purpleScorePos = new Pose();
        Pose parkPos = new Pose();

        switch (side) {
            case LEFT:
                prePurplePos =  new Pose(-15, 35, Math.PI);
                purpleScorePos = new Pose(-15, 35, Math.PI);
                preYellowScorePos = new Pose(-28, 21, Math.PI);
                yellowScorePos = new Pose(-35.5, 21, Math.PI);
                parkPos = new Pose(-27.5, 5, Math.PI);
                break;
            case CENTER:
                prePurplePos = new Pose(-6, 38, Math.PI);
                purpleScorePos = new Pose(-6, 38, Math.PI);
                preYellowScorePos = new Pose(-28, 27, Math.PI);
                yellowScorePos = new Pose(-35.5, 27, Math.PI);
                parkPos = new Pose(-27.5, 5, Math.PI);
                break;
            case RIGHT:
                prePurplePos = new Pose(-2, 29, Math.PI);
                purpleScorePos = new Pose(7, 29, Math.PI);
                preYellowScorePos = new Pose(-28, 32, Math.PI);
                yellowScorePos = new Pose(-35.5, 32, Math.PI);
                parkPos = new Pose(-27.5, 5, Math.PI);
                break;
            default:
                break;

        }

        robot.startIMUThread(this);
        robot.reset();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        //prepurple for right only
                        new swervePositionCommand(drivetrain, localizer, prePurplePos, prePurpleOverride, robot.getVoltage())
                                .alongWith(new MoveArmCommand(lift, deposit, LiftSubsystem.LiftStateReel.ROW1)),

                        //go to purple pos
                        new swervePositionCommand(drivetrain, localizer, purpleScorePos, purpleOverride, robot.getVoltage()),

                        //score purple pixel
                        new IntakeCommand(intake, IntakeSubsystem.IntakeState.AUTON_OUTWARDS)
                                .alongWith(new WaitCommand(intakeScoreLength)),
                        new IntakeCommand(intake, IntakeSubsystem.IntakeState.OFF),

                        //go to pre yellow pos
                        new swervePositionCommand(drivetrain, localizer, preYellowScorePos, preYellowOverride, robot.getVoltage())
                                .alongWith(new InstantCommand( () -> lift.setTargetPos(Globals.ROW1_POS-bucketHeightOffset))),

                        //go to yellow pos
                        new swervePositionCommand(drivetrain, localizer, yellowScorePos, yellowOverride, robot.getVoltage()),

                        //score yellow
                        new GateCommand(deposit, DepositSubsystem.GateState.OPEN),
                        new WaitCommand(scoreDelay),

                        //park
                        new swervePositionCommand(drivetrain, localizer, parkPos, parkOverride, robot.getVoltage())
                                .alongWith(new CancelableResetArmCommand(lift, deposit))

                )
        );
    }

    @Override
    public void run() {

        robot.read(drivetrain, lift);

        CommandScheduler.getInstance().run();

        robot.loop(null, drivetrain, lift);
        localizer.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();

        robot.write(drivetrain, lift);
        robot.clearBulkCache();
    }
}
