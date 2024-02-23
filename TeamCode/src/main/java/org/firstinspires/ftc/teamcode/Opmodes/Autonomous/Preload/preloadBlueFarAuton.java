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
import com.outoftheboxrobotics.photoncore.Photon;
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
@Photon
@Config
@Autonomous(name = "\uD83D\uDD35 ⇒ Preload Far Auto")
public class preloadBlueFarAuton extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    private PropPipeline propPipeline;
    private VisionPortal portal;

    private double loopTime = 0.0;

    public static int scoreDelay = 500;
    public static int pauseDelay = 10000;

    public static int intakeScoreLength = 500;

    public static double preYellowPosX = -78;
    public static double preYellowPosY= 37;
    public static double preYellowPosH = Math.PI;

    public static double yellowPosX = -88;
    public static double yellowPosY= 37;
    public static double yellowPosH = Math.PI;

    public static double purplePosX = -3;
    public static double purplePosY= 30;

    public static double purplePosH = 0;

    public static double parkPosX = -80;
    public static double parkPosY= 55;

    public static double parkPosH = 0;

    public static double gatePosX = -7.5;
    public static double gatePosY= 60;

    public static double gatePosX2 = 15;
    public static double gatePosY2 = 30;

    public static double crossPosX = -65;
    public static double crossPosY= 60;

    public static int preYellowOverride = 5000;//4000;
    public static int yellowOverride = 1000;
    public static int gateOverride = 1000;
    public static int gate2Override = 0;
    public static int purpleOverride = 2500;
    public static int crossOverride = 2000;
    public static int parkOverride = 5000;//2500;

    public static int bucketHeightOffset = 0;

    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();

        Globals.ALLIANCE = Location.BLUE;
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
        Pose gatePos2 = new Pose(gatePosX2, gatePosY2, Math.PI);
        Pose gatePos = new Pose(gatePosX, gatePosY, Math.PI);
        Pose crossPos = new Pose(crossPosX, crossPosY, Math.PI);

        switch (Globals.RANDOMIZATION) {
            case LEFT:
                prePurpleScorePos = new Pose(0, 26, 0);
                purpleScorePos = new Pose(-6, 26, 0);
                gatePos2 = new Pose(gatePosX2, gatePosY2, Math.PI);
                preYellowScorePos = new Pose(-78, 33, Math.PI);
                yellowScorePos = new Pose(-88, 33, Math.PI);
                parkPos = new Pose(parkPosX, parkPosY, Math.PI); //y: 10 for other park
                gate2Override = 1000;
                break;
            case CENTER:
                purpleScorePos = new Pose(6, 38, 0);
                gatePos2 = new Pose(12, 38, 0);
                gatePos = new Pose(10, 60, Math.PI);
                preYellowScorePos = new Pose(-78, preYellowPosY, Math.PI);
                yellowScorePos = new Pose(-88, yellowPosY, Math.PI);
                parkPos = new Pose(parkPosX, parkPosY, Math.PI); //y: 10 for other park
                //gate2Override = 1000;
                //gateOverride = 1000;
                break;
            case RIGHT:
                purpleScorePos = new Pose(10, 40, Math.PI/2);
                gatePos2 = new Pose(gatePosX, gatePosY, Math.PI);
                preYellowScorePos = new Pose(-78, 36, Math.PI);
                yellowScorePos = new Pose(-88, 36, Math.PI);
                parkPos = new Pose(parkPosX, parkPosY, Math.PI); //y: 10 for other park
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
                                .alongWith(new CancelableResetArmCommand()),

                        new swervePositionCommand(new Pose(parkPosX, parkPosY, -Math.PI/2), parkOverride)


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
