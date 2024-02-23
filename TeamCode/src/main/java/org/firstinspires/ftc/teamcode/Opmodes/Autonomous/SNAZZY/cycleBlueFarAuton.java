package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.SNAZZY;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.GateCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.auton.CancelableResetArmCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.auton.swervePositionCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.teleop.MoveArmCommand;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Common.Vision.Location;
import org.firstinspires.ftc.teamcode.Common.Vision.Pipelines.PropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

@Disabled
@Config
@Autonomous(name = "\uD83D\uDD35 Cycle Far Auto")
public class cycleBlueFarAuton extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    private PropPipeline propPipeline;
    private VisionPortal portal;


    private double loopTime = 0.0;

    public static int scoreDelay = 500;
    public static int pauseDelay = 0;

    public static int intakeScoreLength = 500;
    public static int intakingLength = 5000;

    public static double preYellowPosX = -78;
    public static double preYellowPosY= 28;
    public static double preYellowPosH = Math.PI;

    public static double yellowPosX = -88;
    public static double yellowPosY= 28;
    public static double yellowPosH = Math.PI;

    public static double purplePosX = -3;
    public static double purplePosY= 30;

    public static double purplePosH = 0;

    public static double parkPosX = -80;
    public static double parkPosY= 45;

    public static double parkPosH = 0;

    public static double gatePosX = 10;
    public static double gatePosY= 60;

    public static double gatePosX2 = 15;
    public static double gatePosY2 = 30;

    public static double crossPosX = -65;
    public static double crossPosY= 60;

    public static double preStack1X = 13;
    public static double preStack1Y= 26;

    public static double stack1X = 22;
    public static double stack1Y= 26;

    public static int preYellowOverride = 4000;
    public static int yellowOverride = 1000;
    public static int gateOverride = 1000;
    public static int gate2Override = 0;
    public static int purpleOverride = 2500;
    public static int crossOverride = 1350;
    public static int parkOverride = 2500;
    public static int preStack1Override = 5000;
    public static int stack1Override = 1000;

    public static int bucketHeightOffset = 0;

    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();

        Globals.ALLIANCE = Location.BLUE;
        Globals.SIDE = Location.RIGHT;
        Globals.USE_WHEEL_FEEDFORWARD = true;

        robot.init(hardwareMap, telemetry);
        robot.enabled = true;

        robot.localizer.setPoseEstimate(new Pose2d(0, 0, 0));

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

        robot.deposit.update(DepositSubsystem.DepositState.INTAKE);
        robot.deposit.update(DepositSubsystem.GateState.OPEN);

        robot.read();
        while (!isStarted()) {
            robot.read();
            robot.drivetrain.frontLeftModule.setTargetRotation(0);
            robot.drivetrain.frontRightModule.setTargetRotation(0);
            robot.drivetrain.backRightModule.setTargetRotation(0);
            robot.drivetrain.backLeftModule.setTargetRotation(0);
            robot.drivetrain.updateModules();

            telemetry.addLine("auto in init");
            telemetry.addData("POS", propPipeline.getLocation());
            telemetry.update();

            robot.clearBulkCache();
            robot.write();
        }

        robot.localizer.setPoseEstimate(new Pose2d(0, 0, 0));

        Location side = propPipeline.getLocation();
        portal.close();

        Pose yellowScorePos = new Pose();
        Pose preYellowScorePos = new Pose();
        Pose prePurpleScorePos = new Pose(0, 0, 0);
        Pose purpleScorePos = new Pose();
        Pose parkPos = new Pose();
        Pose gatePos2 = new Pose(gatePosX2, gatePosY2, Math.PI);
        Pose gatePos = new Pose(gatePosX, gatePosY, Math.PI);
        Pose crossPos = new Pose(crossPosX, crossPosY, Math.PI);
        Pose preStack1Pos = new Pose(preStack1X, preStack1Y, Math.PI);
        Pose stack1Pos = new Pose(stack1X, stack1Y, Math.PI);

        switch (side) {
            case LEFT:
                prePurpleScorePos = new Pose(-3, 30, 0);
                purpleScorePos = new Pose(-6, 30, 0);
                gatePos2 = new Pose(gatePosX2, gatePosY2, Math.PI);
                preYellowScorePos = new Pose(-80, 28, Math.PI);
                yellowScorePos = new Pose(-89.5, 28, Math.PI);
                parkPos = new Pose(-80, 52, Math.PI); //y: 5 for other park
                gate2Override = 1000;
                break;
            case CENTER:
                purpleScorePos = new Pose(0, 39, -Math.PI/2);
                gatePos2 = new Pose(gatePosX, gatePosY, Math.PI);
                preYellowScorePos = new Pose(-80, 31, Math.PI);
                yellowScorePos = new Pose(-89.5, 31, Math.PI);
                parkPos = new Pose(-80, 52, Math.PI); //y: 5 for other park
                purpleOverride = 5000;
                gate2Override = 0;
                break;
            case RIGHT:
                purpleScorePos = new Pose(9, 35, -Math.PI/2);
                gatePos2 = new Pose(gatePosX, gatePosY, Math.PI);
                preYellowScorePos = new Pose(-80, 36, Math.PI);
                yellowScorePos = new Pose(-89.5, 36, Math.PI);
                parkPos = new Pose(-80, 52, Math.PI); //y: 5 for other park
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
                        new swervePositionCommand(prePurpleScorePos, 1000),

                        // go to purple pixel scoring pos
                        new swervePositionCommand(purpleScorePos, purpleOverride),

                        // score purple pixel
                        new IntakeCommand(IntakeSubsystem.IntakeState.OUTWARDS)
                                .alongWith(new WaitCommand(intakeScoreLength)),
                        new IntakeCommand(IntakeSubsystem.IntakeState.OFF),

                        //prestack1 pos
                        new swervePositionCommand(preStack1Pos, preStack1Override),

                        //stack1 pos
                        new swervePositionCommand(stack1Pos, stack1Override),

                        //intake white
                        new IntakeCommand(IntakeSubsystem.IntakeState.INWARDS)
                                .alongWith(new WaitCommand(intakingLength)),
                        new IntakeCommand(IntakeSubsystem.IntakeState.OUTWARDS),

                        //gate pos2
                        new swervePositionCommand(gatePos2, gate2Override),

                        //gate pos
                        new swervePositionCommand(gatePos, gateOverride),

                        //after gate pos
                        new swervePositionCommand(crossPos, crossOverride),

                        //wait for other bots
                        new WaitCommand(pauseDelay)
                                .alongWith(new IntakeCommand(IntakeSubsystem.IntakeState.OFF)),

                        //pre yellow pos
                        new swervePositionCommand(preYellowScorePos, preYellowOverride)
                                .alongWith(new MoveArmCommand(LiftSubsystem.LiftStateReel.ROW1)),

                        //go to yellow scoring pos
                        new swervePositionCommand(yellowScorePos, yellowOverride)
                                .alongWith(new InstantCommand( () -> robot.lift.setTargetPos(Globals.ROW1_POS-bucketHeightOffset))),

                        //score yellow
                        new GateCommand(DepositSubsystem.GateState.OPEN),
                        new WaitCommand(scoreDelay),

                        //go to park pos
                        new swervePositionCommand(parkPos, parkOverride)
                                .alongWith(new CancelableResetArmCommand())


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
        loopTime = loop;
        telemetry.update();

        CommandScheduler.getInstance().run();

    }
}
