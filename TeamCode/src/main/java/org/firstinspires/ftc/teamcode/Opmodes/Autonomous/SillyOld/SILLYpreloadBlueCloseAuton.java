package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.SillyOld;

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
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.LiftCommand;
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
@Autonomous(name = "Blue Close Auto")
public class SILLYpreloadBlueCloseAuton extends CommandOpMode {

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

    public static double preYellowPosX = -24;
    public static double preYellowPosY= 35;
    public static double preYellowPosH = Math.PI;

    public static double yellowPosX = -32.5;
    public static double yellowPosY= 35;
    public static double yellowPosH = Math.PI;

    public static double purplePosX = 6;
    public static double purplePosY= 32;

    public static double purplePosH = 0;

    public static double parkPosX = -36;
    public static double parkPosY= 5;

    public static double parkPosH = 0;

    public static double gatePosX = -7.5;
    public static double gatePosY= 60;

    public static double preStackPosX = 68;
    public static double preStackPosY= 60;

    public static double stackPosX = 75;
    public static double stackPosY= 52.5;

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
        Pose purpleScorePos = new Pose();
        Pose parkPos = new Pose();

        Pose preYellowScorePosH = new Pose();


        // 0.3, 300

        switch (side) {
            case LEFT:
                preYellowScorePosH = new Pose(-24, 22.5, Math.PI + Math.toRadians(hLeOffset));
                yellowScorePos = new Pose(-33, 22.5, Math.PI + Math.toRadians(hLeOffset));
                purpleScorePos = new Pose(-15, 35, Math.PI);
                parkPos = new Pose(-27.5, 5, Math.PI);
                break;
            case CENTER:
                preYellowScorePosH = new Pose(-24, 27, Math.PI + Math.toRadians(hCeOffset));
                yellowScorePos = new Pose(-33, 27, Math.PI + Math.toRadians(hCeOffset));
                purpleScorePos = new Pose(-6, 38, Math.PI);
                parkPos = new Pose(-27.5, 5, Math.PI);
                break;
            case RIGHT:
                preYellowScorePosH = new Pose(-24, 34, Math.PI + Math.toRadians(hRiOffset));
                yellowScorePos = new Pose(-33, 34, Math.PI + Math.toRadians(hRiOffset));
                purpleScorePos = new Pose(6, 32.5, Math.PI);
                parkPos = new Pose(-27.5, 5, Math.PI);
                break;
            default:
                break;

        }

        robot.startIMUThread(this);
        robot.reset();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new swervePositionCommand(drivetrain, localizer, preYellowScorePosH, robot.getVoltage())
                                .alongWith(new MoveArmCommand(lift, deposit, LiftSubsystem.LiftStateReel.ROW3)),

                        //go to yellow scoring pos
                        new swervePositionCommand(drivetrain, localizer, yellowScorePos, robot.getVoltage())
                                .alongWith(new MoveArmCommand(lift, deposit, LiftSubsystem.LiftStateReel.ROW2)),

                        new InstantCommand(() -> deposit.update(DepositSubsystem.DepositState.DEPOSIT)),
                        new WaitCommand(unscoreDelay),
                        new InstantCommand(() -> deposit.update(DepositSubsystem.DepositState.HANG))
                            .andThen(new MoveArmCommand(lift, deposit, LiftSubsystem.LiftStateReel.ROW5)),

                        //Manual Arm Reset
                        new DepositCommand(deposit, DepositSubsystem.DepositState.INTERMEDIATE)
                                .alongWith(new LiftCommand(lift, LiftSubsystem.LiftStateReel.ROW1)),
                        new WaitCommand(Globals.LIFT_DELAY),
                        new DepositCommand(deposit, DepositSubsystem.DepositState.DEPOSIT),
                        new WaitCommand(Globals.FLIP_IN_DELAY),
                        new LiftCommand(lift, LiftSubsystem.LiftStateReel.DOWN),
                        new DepositCommand(deposit, DepositSubsystem.DepositState.INTAKE),

                        // go to purple pixel scoring pos
                        new swervePositionCommand(drivetrain, localizer, purpleScorePos, robot.getVoltage()),

                        // score purple pixel
                        new IntakeCommand(intake, IntakeSubsystem.IntakeState.AUTON_OUTWARDS)
                                .alongWith(new WaitCommand(intakeScoreLength)),
                        new IntakeCommand(intake, IntakeSubsystem.IntakeState.OFF),

                        //go to park pos
                        new swervePositionCommand(drivetrain, localizer, parkPos, robot.getVoltage())

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
