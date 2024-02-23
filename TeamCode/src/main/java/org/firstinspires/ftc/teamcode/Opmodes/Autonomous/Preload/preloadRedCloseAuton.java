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
@Autonomous(name = "\uD83D\uDFE5 ⇒ Preload Close Auto")
public class preloadRedCloseAuton extends CommandOpMode {

    public static int scoreDelay = 500;
    public static int intakeScoreLength = 750;
    public static double preYellowPosX = -28;
    public static double preYellowPosY= 35;
    public static double preYellowPosH = Math.PI;
    public static double yellowPosX = -37;
    public static double yellowPosY= 35;
    public static double yellowPosH = Math.PI;
    public static double purplePosX = -7;
    public static double purplePosY= 32;
    public static double purplePosH = 0;
    public static double parkPosX = 30;
    public static double parkPosY= 5;
    public static double parkPosH = 0;
    public static int preYellowOverride = 5000;//2000;
    public static int yellowOverride = 1000;
    public static int prePurpleOverride = 5000;//1500;
    public static int purpleOverride = 5000;//2500;
    public static int parkOverride = 2500;
    public static int bucketHeightOffset = 0;

    private final RobotHardware robot = RobotHardware.getInstance();

    private PropPipeline propPipeline;
    private VisionPortal portal;
    private double loopTime = 0.0;

    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();

        Globals.ALLIANCE = Location.RED;
        Globals.SIDE = Location.CLOSE;
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

        Pose preYellowScorePos = new Pose();
        Pose yellowScorePos = new Pose();
        Pose prePurplePos = new Pose();
        Pose purpleScorePos = new Pose();
        Pose parkPos = new Pose();

        switch (Globals.RANDOMIZATION) {
            case LEFT:
                prePurplePos =  new Pose(6, 23, -Math.PI/2);
                purpleScorePos = new Pose(6, 23, -Math.PI/2);
                preYellowScorePos = new Pose(30, 21, 0);
                yellowScorePos = new Pose(38, 21, 0);
                parkPos = new Pose(parkPosX, parkPosY, 0); //y: 52 for other park
                break;
            case CENTER:
                prePurplePos = new Pose(5 , 37, 0);
                purpleScorePos = new Pose(5, 37, 0);
                preYellowScorePos = new Pose(30, 27 , 0);
                yellowScorePos = new Pose(38, 27, 0);
                parkPos = new Pose(parkPosX, parkPosY, 0); //y: 52 for other park
                break;
            case RIGHT:
                prePurplePos = new Pose(2, 27, 0);
                purpleScorePos = new Pose(-4, 27, 0);
                preYellowScorePos = new Pose(30, 33, 0);
                yellowScorePos = new Pose(38, 33, 0);
                parkPos = new Pose(parkPosX, parkPosY, 0); //y: 52 for other park
                break;
            default:
                break;

        }

        robot.startIMUThread(this);
        robot.reset();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        //prepurple for right only
                        new swervePositionCommand(prePurplePos, prePurpleOverride)
                                .alongWith(new MoveArmCommand(LiftSubsystem.LiftStateReel.ROW1)),

                        //go to purple pos
                        new swervePositionCommand(purpleScorePos, purpleOverride),

                        //score purple pixel
                        new IntakeCommand(IntakeSubsystem.IntakeState.OUTWARDS)
                                .alongWith(new WaitCommand(intakeScoreLength)),
                        new IntakeCommand(IntakeSubsystem.IntakeState.OFF),

                        //go to pre yellow pos
                        new swervePositionCommand(preYellowScorePos, preYellowOverride)
                                .alongWith(new InstantCommand( () -> robot.lift.setTargetPos(Globals.ROW1_POS-bucketHeightOffset))),

                        //go to yellow pos
                        new swervePositionCommand(yellowScorePos, yellowOverride),

                        //score yellow
                        new GateCommand(DepositSubsystem.GateState.OPEN),
                        new WaitCommand(scoreDelay),

                        //park
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
