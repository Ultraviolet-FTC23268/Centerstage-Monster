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
@Autonomous(name = "\uD83D\uDFE5 ⇐ Preload Far Auto")
public class preloadRedFarAuton extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
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

        Globals.ALLIANCE = Location.RED;
        Globals.SIDE = Location.FAR;
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
        robot.deposit.update(DepositSubsystem.GateState.CLOSED);

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

        Globals.RANDOMIZATION = propPipeline.getLocation();
        portal.close();

        Pose yellowScorePos = new Pose();
        Pose preYellowScorePos = new Pose();
        Pose prePurpleScorePos = new Pose(0, 0, 0);
        Pose purpleScorePos = new Pose();
        Pose parkPos = new Pose();
        Pose gatePos2 = new Pose(gatePosX2, gatePosY2, 0);
        Pose gatePos = new Pose(gatePosX, gatePosY, 0);
        Pose crossPos = new Pose(crossPosX, crossPosY, 0);

        switch (Globals.RANDOMIZATION) {
            case LEFT:
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
            case RIGHT:
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
                        new swervePositionCommand(prePurpleScorePos, 1000),

                        // go to purple pixel scoring pos
                        new swervePositionCommand(purpleScorePos, purpleOverride),

                        // score purple pixel
                        new IntakeCommand(IntakeSubsystem.IntakeState.OUTWARDS)
                                .alongWith(new WaitCommand(intakeScoreLength)),
                        new IntakeCommand(IntakeSubsystem.IntakeState.OFF),

                        //gate pos2

                        new swervePositionCommand(gatePos2, gate2Override),

                        //gate pos
                        new swervePositionCommand(gatePos, gateOverride),

                        //after gate pos
                        new swervePositionCommand(crossPos, crossOverride),

                        //wait for other bots
                        new WaitCommand(pauseDelay),

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
