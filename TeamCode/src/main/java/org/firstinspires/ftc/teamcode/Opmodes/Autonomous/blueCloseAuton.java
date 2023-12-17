package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

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
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.auton.PositionCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.auton.swervePositionCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.teleop.MoveArmCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.teleop.ResetArmCommand;
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
@Autonomous(name = "Close Blue Auto")
public class blueCloseAuton extends CommandOpMode {

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
        Pose preYellowScorePos = new Pose();
        Pose purpleScorePos = new Pose();
        Pose parkPos = new Pose();
        Pose gatePos = new Pose(gatePosX, gatePosY, Math.PI);
        Pose preStackPos = new Pose(preStackPosX, preStackPosY, Math.PI);
        Pose stackPos = new Pose(stackPosX, stackPosY, Math.PI);

        Pose yellowScorePosH = new Pose();
        Pose preYellowScorePosH = new Pose();
        Pose purpleScorePosH = new Pose();
        Pose parkPosHeading = new Pose();



        // 0.3, 300

        switch (side) {
            case LEFT:
                preYellowScorePos = new Pose(-24, 19.5,0);
                preYellowScorePosH = new Pose(-24, 19.5, Math.PI);
                yellowScorePos = new Pose(-32.5, 19.5, Math.PI);
                purpleScorePos = new Pose(-16, 35, Math.PI);
                parkPos = new Pose(-24, 5, Math.PI);
                break;
            case CENTER:
                preYellowScorePos = new Pose(-24, 26.5, 0);
                preYellowScorePosH = new Pose(-24, 26.5, Math.PI);
                yellowScorePos = new Pose(-32.5, 26.5, Math.PI);
                purpleScorePos = new Pose(-6, 38, Math.PI);
                parkPos = new Pose(-24, 5, Math.PI);
                break;
            case RIGHT:
                preYellowScorePos = new Pose(-24, 32.5,0);
                preYellowScorePosH = new Pose(-24, 34, Math.PI);
                yellowScorePos = new Pose(-33.5, 34, Math.PI);
                purpleScorePos = new Pose(6, 32.5, Math.PI);
                parkPos = new Pose(-24, 5, Math.PI);
                break;
            default:
                break;

        }

        robot.startIMUThread(this);
        robot.reset();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new swervePositionCommand(drivetrain, localizer, preYellowScorePosH,12.5)
                                .alongWith(new MoveArmCommand(lift, deposit, LiftSubsystem.LiftStateReel.ROW3)),

                        //go to yellow scoring pos
                        new swervePositionCommand(drivetrain, localizer, yellowScorePos, 12.5)
                                .alongWith(new MoveArmCommand(lift, deposit, LiftSubsystem.LiftStateReel.ROW1)),

                        new InstantCommand(() -> deposit.update(DepositSubsystem.DepositState.DEPOSIT3)),
                        new WaitCommand(unscoreDelay),
                        new InstantCommand(() -> deposit.update(DepositSubsystem.DepositState.HANG))
                            .andThen(new MoveArmCommand(lift, deposit, LiftSubsystem.LiftStateReel.ROW5)),

                        // go to purple pixel scoring pos
                        new swervePositionCommand(drivetrain, localizer, purpleScorePos, 12.5)
                            .andThen(new ResetArmCommand(lift, deposit)),

                        // score purple pixel
                        new IntakeCommand(intake, IntakeSubsystem.IntakeState.AUTON_OUTWARDS)
                                .alongWith(new WaitCommand(intakeScoreLength)),
                        new IntakeCommand(intake, IntakeSubsystem.IntakeState.OFF),



                        //go to pos in front of gate
                        new swervePositionCommand(drivetrain, localizer, gatePos, 12.5),

                        //go to pre-stack pos
                        new swervePositionCommand(drivetrain, localizer, preStackPos, 12.5),

                        //go to stack pos
                        new swervePositionCommand(drivetrain, localizer, stackPos, 12.5),

                        //go back under gate
                        new swervePositionCommand(drivetrain, localizer, gatePos, 12.5),

                        //go to park pos
                        new swervePositionCommand(drivetrain, localizer, parkPos, 12.5)


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
