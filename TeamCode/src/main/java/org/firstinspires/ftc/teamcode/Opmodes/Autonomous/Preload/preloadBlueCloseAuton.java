package org.firstinspires.ftc.teamcode.Opmodes.Autonomous.Preload;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.GateCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.auton.CancelableResetArmCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.auton.RelocalizeCommand;
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

@Photon
@Config
@Autonomous(name = "\uD83D\uDD35 ⇐ Preload Close Auto")
public class preloadBlueCloseAuton extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    private PropPipeline propPipeline;
    private VisionPortal portal;
    private Location randomization;

    private double loopTime = 0.0;

    public static int scoreDelay = 500;

    public static int prePurpleOverride = 5000;
    public static int purpleOverride = 1500;
    public static int purpleBackOverride = 1000;
    public static int backdropOverride = 5000;
    public static int yellowOverride = 5000;
    public static int parkOverride = 5000;

    public static double hOffset = -5;

    public static double pBackupDistance = 5;

    //Position Constants

    public static double pPoseX = -45;
    public static double pPoseY = 18;

    public static double prePoseY = 16;

    public static double pbPoseX = -55;
    public static double pbPoseY = 18;

    public static double yPoseX = -28;
    public static double yPoseY = 51;

    public static double bPoseX = -40;
    public static double bPoseY = 40;

    public static double parkPoseX = -61;
    public static double parkPoseY = 46;

    @Override
    public void initialize() {

        CommandScheduler.getInstance().reset();

        Globals.ALLIANCE = Location.BLUE;
        Globals.SIDE = Location.CLOSE;
        Globals.USE_WHEEL_FEEDFORWARD = true;
        Globals.AUTO = true;

        robot.init(hardwareMap, telemetry);
        robot.enabled = true;

        robot.localizer.setPoseEstimate(new Pose2d());

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.deposit.update(DepositSubsystem.DepositState.INTAKE);
        robot.deposit.update(DepositSubsystem.GateState.CLOSED);
        robot.purpleLatch.setPosition(Globals.PURPLE_CLOSED);

        propPipeline = new PropPipeline();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(propPipeline)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        while (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("initializing... please wait");
            telemetry.update();
            if(isStopRequested()) portal.close();
        }

        while (!isStarted()) {
            if(isStopRequested()) portal.close();

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


        robot.localizer.setPoseEstimate(new Pose2d(62, -16, 0));

        randomization = propPipeline.getLocation();
        Globals.RANDOMIZATION = randomization;
        portal.close();

        RobotHardware.getInstance().preloadDetectionPipeline.setTargetAprilTagID(randomization);

        Pose parkPos = new Pose(parkPoseX, parkPoseY, Math.PI +Math.toRadians(hOffset));

        Pose backdropPos = new Pose();
        Pose prePurplePos = new Pose();
        Pose purpleScorePos = new Pose();
        Pose purpleBackPos = new Pose();
        Pose yellowScorePos = new Pose();

        switch (randomization) {
            case LEFT:
                prePurplePos = new Pose(-32, 24, 0 +Math.toRadians(hOffset));
                purpleScorePos = new Pose(-32, 24, 0 +Math.toRadians(hOffset));
                purpleBackPos = new Pose(-52, 24, 0 +Math.toRadians(hOffset));
                yellowScorePos = new Pose(-29, 51, Math.PI/2 +Math.toRadians(hOffset));
                backdropPos = new Pose(-28, 45, Math.PI/2 +Math.toRadians(hOffset));
                hOffset--;
                break;
            case CENTER:
                prePurplePos = new Pose(-26, 21, -Math.PI/2 +Math.toRadians(hOffset));
                purpleScorePos = new Pose(-26, 21, -Math.PI/2 +Math.toRadians(hOffset));
                purpleBackPos = new Pose(-26, 26, -Math.PI/2 +Math.toRadians(hOffset));
                yellowScorePos = new Pose(-35, 51, Math.PI/2 +Math.toRadians(hOffset));
                backdropPos = new Pose(-30, 46, Math.PI/2 +Math.toRadians(hOffset));
                break;
            case RIGHT:
                prePurplePos = new Pose(-34, 18, -Math.PI/2 +Math.toRadians(hOffset));
                purpleScorePos = new Pose(-34, 10, -Math.PI/2 +Math.toRadians(hOffset));
                purpleBackPos = new Pose(-34, 18, -Math.PI/2 +Math.toRadians(hOffset));
                yellowScorePos = new Pose(-42, 51, Math.PI/2 +Math.toRadians(hOffset));
                backdropPos = new Pose(-35, 47, Math.PI/2 +Math.toRadians(hOffset));
                hOffset++;
                break;
            default:
                break;

        }

        robot.startIMUThread(this);
        robot.reset();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new InstantCommand(() -> robot.startCamera()),

                        new swervePositionCommand(prePurplePos, prePurpleOverride),

                        //purple pos
                        new swervePositionCommand(purpleScorePos, purpleOverride),

                        //drop purple
                        new InstantCommand(() -> robot.purpleLatch.setPosition(Globals.PURPLE_OPEN)),
                        new WaitCommand(500),

                        //back up
                        new swervePositionCommand(purpleBackPos, purpleBackOverride)
                                .alongWith(new MoveArmCommand(LiftSubsystem.LiftStateReel.AUTO)),

                        //backdrop pos
                        new swervePositionCommand(backdropPos, backdropOverride),

                        //relocalize
                        new RelocalizeCommand(),
                        new InstantCommand(() -> robot.closeCamera()),

                        //yellow pos
                        new swervePositionCommand(yellowScorePos, yellowOverride),

                        //score yellow
                        new GateCommand(DepositSubsystem.GateState.OPEN),
                        new WaitCommand(scoreDelay),

                        new swervePositionCommand(new Pose(yellowScorePos.getX(), (yellowScorePos.getY()-8), Math.PI/2), 2000),

                        //park
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
        telemetry.addData("BACKDROP", RobotHardware.getInstance().preloadDetectionPipeline.getPreloadedZone());
        loopTime = loop;
        telemetry.update();

        CommandScheduler.getInstance().run();

    }
}
