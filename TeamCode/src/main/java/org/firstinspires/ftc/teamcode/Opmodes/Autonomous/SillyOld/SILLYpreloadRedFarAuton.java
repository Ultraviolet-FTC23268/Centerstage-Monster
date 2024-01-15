package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.SillyOld;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.auton.swervePositionCommand;
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
@Autonomous(name = "Red Far Auto")
public class SILLYpreloadRedFarAuton extends CommandOpMode {

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
    public static int unscoreDelay = 1000;

    public static int intakeScoreLength = 750;

    public static double preYellowPosX = -75;
    public static double preYellowPosY= 22;
    public static double preYellowPosH = Math.PI;

    public static double yellowPosX = -86;
    public static double yellowPosY= 22;
    public static double yellowPosH = Math.PI;

    public static double purplePosX = -3;
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
    public static double crossPosY= 60;


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
        Pose purpleScorePos = new Pose();
        Pose parkPos = new Pose();
        Pose gatePos2 = new Pose(gatePosX2, gatePosY2, 0);
        Pose gatePos = new Pose(gatePosX, gatePosY, 0);
        Pose crossPos = new Pose(crossPosX, crossPosY, 0);

        Pose preYellowScorePosH = new Pose();

        // 0.3, 300

        switch (side) {
            case RIGHT:
                preYellowScorePosH = new Pose(75, 24, 0);
                yellowScorePos = new Pose(85.5, 24, 0);
                purpleScorePos = new Pose(3, 30, 0);
                parkPos = new Pose(80, 45, 0);
                gatePos2 = new Pose(gatePosX2, gatePosY2, 0);
                break;
            case CENTER:
                preYellowScorePosH = new Pose(75, 27.5, 0);
                yellowScorePos = new Pose(85.5, 27.5, 0);
                purpleScorePos = new Pose(0, 46, Math.PI/2);
                parkPos = new Pose(80, 45, 0);
                gatePos2 = new Pose(gatePosX, gatePosY, 0);
                break;
            case LEFT:
                preYellowScorePosH = new Pose(75, 35, 0);
                yellowScorePos = new Pose(85.5, 35, 0);
                purpleScorePos = new Pose(9, 40, Math.PI/2);
                parkPos = new Pose(80, 45, 0);
                gatePos2 = new Pose(gatePosX, gatePosY, 0);
                break;
            default:
                break;

        }

        robot.startIMUThread(this);
        robot.reset();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        // go to purple pixel scoring pos
                        new swervePositionCommand(drivetrain, localizer, purpleScorePos, 12.5),

                        // score purple pixel
                        new IntakeCommand(intake, IntakeSubsystem.IntakeState.AUTON_OUTWARDS)
                                .alongWith(new WaitCommand(intakeScoreLength)),
                        new IntakeCommand(intake, IntakeSubsystem.IntakeState.OFF)
/*
                        //gate pos2

                        new swervePositionCommand(drivetrain, localizer, gatePos2, 12.5),

                        //gate pos
                        new swervePositionCommand(drivetrain, localizer, gatePos, 12.5),

                        //after gate pos
                        new swervePositionCommand(drivetrain, localizer, crossPos, 12.5),

                        //pre yellow pos
                        new swervePositionCommand(drivetrain, localizer, preYellowScorePosH,12.5)
                                .alongWith(new MoveArmCommand(lift, deposit, LiftSubsystem.LiftStateReel.ROW3)),

                        //go to yellow scoring pos
                        new swervePositionCommand(drivetrain, localizer, yellowScorePos, 12.5)
                                .alongWith(new MoveArmCommand(lift, deposit, LiftSubsystem.LiftStateReel.ROW1)),

                        //score yellow
                        new InstantCommand(() -> deposit.update(DepositSubsystem.DepositState.AUTON)),
                        new WaitCommand(unscoreDelay),
                        new InstantCommand(() -> deposit.update(DepositSubsystem.DepositState.HANG))
                            .andThen(new MoveArmCommand(lift, deposit, LiftSubsystem.LiftStateReel.ROW5)),

                        //Manual Arm Reset
                        new DepositCommand(deposit, DepositSubsystem.DepositState.INTERMEDIATE)
                                .alongWith(new LiftCommand(lift, LiftSubsystem.LiftStateReel.ROW1)),
                        new WaitCommand(Globals.LIFT_DELAY),
                        new DepositCommand(deposit, DepositSubsystem.DepositState.INTERMEDIATE2),
                        new WaitCommand(Globals.FLIP_IN_DELAY),
                        new LiftCommand(lift, LiftSubsystem.LiftStateReel.DOWN),
                        new DepositCommand(deposit, DepositSubsystem.DepositState.INTAKE),

                        //go to park pos
                        new swervePositionCommand(drivetrain, localizer, parkPos, 12.5)

*/
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
        telemetry.addData("xPos ", localizer.getPos().x);
        telemetry.addData("yPos ", localizer.getPos().y);
        telemetry.addData("hPos ", localizer.getPos().heading);
        loopTime = loop;
        telemetry.update();

        robot.write(drivetrain, lift);
        robot.clearBulkCache();
    }
}
