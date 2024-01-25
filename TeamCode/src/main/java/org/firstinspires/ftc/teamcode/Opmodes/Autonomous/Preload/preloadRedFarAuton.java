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
@Autonomous(name = "\uD83D\uDFE5 ⇐ Preload Far Auto")
public class preloadRedFarAuton extends CommandOpMode {

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
    public static int pauseDelay = 10000;

    public static int intakeScoreLength = 500;

    public static double preYellowPosX = 83;
    public static double preYellowPosY= 17;
    public static double preYellowPosH = Math.PI;

    public static double yellowPosX = 90;
    public static double yellowPosY= 17;
    public static double yellowPosH = Math.PI;

    public static double purplePosX = 6;
    public static double purplePosY= 30;

    public static double purplePosH = 0;

    public static double parkPosX = -80;
    public static double parkPosY= 45;

    public static double parkPosH = 0;

    public static double gatePosX = 7.5;
    public static double gatePosY= 60;

    public static double gatePosX2 = -15;
    public static double gatePosY2 = 30;

    public static double crossPosX = 65;
    public static double crossPosY= 60;;

    public static int preYellowOverride = 5000;//4000;
    public static int yellowOverride = 1000;//1000;
    public static int gateOverride = 1500;//1000;
    public static int gate2Override = 0;
    public static int purpleOverride = 5000;//2500;
    public static int crossOverride = 2500;//1500;
    public static int parkOverride = 5000;//2500;

    public static int bucketHeightOffset = 0;

    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();

        Globals.COLOR = Side.RED;
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

        Pose yellowScorePos = new Pose();
        Pose preYellowScorePos = new Pose();
        Pose prePurpleScorePos = new Pose(0, 0, 0);
        Pose purpleScorePos = new Pose();
        Pose parkPos = new Pose();
        Pose gatePos2 = new Pose(gatePosX2, gatePosY2, 0);
        Pose gatePos = new Pose(gatePosX, gatePosY, 0);
        Pose crossPos = new Pose(crossPosX, crossPosY, 0);

        switch (side) {
            case RIGHT:
                prePurpleScorePos = new Pose(0, 32, Math.PI);
                purpleScorePos = new Pose(purplePosX, purplePosY, Math.PI);
                gatePos2 = new Pose(gatePosX2, gatePosY2, 0);
                preYellowScorePos = new Pose(preYellowPosX, preYellowPosY, 0);
                yellowScorePos = new Pose(yellowPosX, yellowPosY, 0);
                parkPos = new Pose(82, 48, -Math.PI/2); //y: 10 for other park
                gate2Override = 1000;
                break;
            case CENTER:
                purpleScorePos = new Pose(-6, 41, Math.PI);
                gatePos2 = new Pose(-15, 45, Math.PI);
                gatePos = new Pose(-10, 60, 0);
                preYellowScorePos = new Pose(83, 23, 0);
                yellowScorePos = new Pose(90, 23, 0);
                parkPos = new Pose(82, 48, -Math.PI/2); //y: 10 for other park
                gate2Override = 1000;
                gateOverride = 1000;
                break;
            case LEFT:
                purpleScorePos = new Pose(-5, 45, Math.PI/2);
                gatePos2 = new Pose(gatePosX, gatePosY, 0);
                preYellowScorePos = new Pose(83, 29, 0);
                yellowScorePos = new Pose(90, 29, 0);
                parkPos = new Pose(82, 48, -Math.PI/2); //y: 10 for other park
                gate2Override = 0;
                break;
            default:
                break;

        }

        robot.startIMUThread(this);
        robot.reset();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        // go to prepurple pixel scoring pos (left only)
                        new swervePositionCommand(drivetrain, localizer, prePurpleScorePos, 1000, robot.getVoltage()),

                        // go to purple pixel scoring pos
                        new swervePositionCommand(drivetrain, localizer, purpleScorePos, purpleOverride, robot.getVoltage()),

                        // score purple pixel
                        new IntakeCommand(intake, IntakeSubsystem.IntakeState.AUTON_OUTWARDS)
                                .alongWith(new WaitCommand(intakeScoreLength)),
                        new IntakeCommand(intake, IntakeSubsystem.IntakeState.OFF),

                        //gate pos2

                        new swervePositionCommand(drivetrain, localizer, gatePos2, gate2Override, robot.getVoltage()),

                        //gate pos
                        new swervePositionCommand(drivetrain, localizer, gatePos, gateOverride, robot.getVoltage()),

                        //after gate pos
                        new swervePositionCommand(drivetrain, localizer, crossPos, crossOverride, robot.getVoltage()),

                        //wait for other bots
                        new WaitCommand(pauseDelay),

                        //pre yellow pos
                        new swervePositionCommand(drivetrain, localizer, preYellowScorePos, preYellowOverride, robot.getVoltage())
                                .alongWith(new MoveArmCommand(lift, deposit, LiftSubsystem.LiftStateReel.ROW1)),

                        //go to yellow scoring pos
                        new swervePositionCommand(drivetrain, localizer, yellowScorePos, yellowOverride, robot.getVoltage())
                                .alongWith(new InstantCommand( () -> lift.setTargetPos(Globals.ROW1_POS-bucketHeightOffset))),

                        //score yellow
                        new GateCommand(deposit, DepositSubsystem.GateState.OPEN),
                        new WaitCommand(scoreDelay),

                        //go to park pos
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
