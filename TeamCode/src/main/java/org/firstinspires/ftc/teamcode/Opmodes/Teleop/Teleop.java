package org.firstinspires.ftc.teamcode.Opmodes.Teleop;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.Commands.abobot.SpeedChangeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.teleop.ClimbUpCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.teleop.EjectCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.teleop.IncrementedMoveArmCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.teleop.MoveArmCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.teleop.ResetArmCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.teleop.DisableAllCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.teleop.ScoreCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.teleop.UnscoreCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.teleop.ConditionalintakeCommand;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.swerve.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.Point;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.localizer.Localizer;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;

//@Photon(maximumParallelCommands = 8)
@Config
@TeleOp(name = "\uD83D\uDCB2\uD83D\uDCB8\uD83E\uDD11")
public class Teleop extends CommandOpMode {
    private ElapsedTime timer;
    private double loopTime = 0;

    private boolean pHeadingLock = true;
    private double targetHeading;

    private final RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private IntakeSubsystem intake;
    private LiftSubsystem lift;
    private DepositSubsystem deposit;

    private SlewRateLimiter fw;
    private SlewRateLimiter str;
    private final PIDFController hController = new PIDFController(0.5, 0, 0.1, 0);

    public static double fw_r = 4;
    public static double str_r = 4;
    private boolean lock_robot_heading = false;


    GamepadEx gamepadEx, gamepadEx2;
    Localizer localizer;

    public static boolean autoGrabActive = false;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Globals.AUTO = false;
        Globals.USING_IMU = true;
        Globals.USE_WHEEL_FEEDFORWARD = false;

        robot.init(hardwareMap, telemetry);
        drivetrain = new SwerveDrivetrain(robot);
        intake = new IntakeSubsystem(robot);
        lift = new LiftSubsystem(robot);
        deposit = new DepositSubsystem(robot);
        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        localizer = new TwoWheelLocalizer(robot);

        robot.enabled = true;
        robot.startIMUThread(this);

        //robot.droneLatch.setPosition(Globals.DRONE_CLOSED);
        //deposit.update(DepositSubsystem.DepositState.INTAKE);

        //robot.LEDcontroller.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);

        //GAMEPAD 1

        //Intake
        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> schedule(new ConditionalintakeCommand(intake, lift, IntakeSubsystem.IntakeState.INWARDS)));
        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenReleased(() -> schedule(new ConditionalintakeCommand(intake, lift, IntakeSubsystem.IntakeState.OFF)));

        //Outtake
        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> schedule(new ConditionalintakeCommand(intake, lift, IntakeSubsystem.IntakeState.OUTWARDS)));
        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
                .whenReleased(() -> schedule(new ConditionalintakeCommand(intake, lift, IntakeSubsystem.IntakeState.OFF)));

        //Slowdown Drivetrain
        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> schedule(new SpeedChangeCommand(Globals.SLOWDOWN_SPEED)));
        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)
                .whenReleased(() -> schedule(new SpeedChangeCommand(1)));

        /*gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> robot.LEDcontroller.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> robot.LEDcontroller.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> robot.LEDcontroller.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> robot.LEDcontroller.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK));*/

        //GAMEPAD 2

        //Reset Lift
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> schedule(new ResetArmCommand(lift, deposit)));

        //Max Lift
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> schedule(new MoveArmCommand(lift, deposit, LiftSubsystem.LiftStateReel.MAX)));

        //Move Up a Row
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> schedule(new IncrementedMoveArmCommand(lift, deposit, 1)));

        //Move Down a Row
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> schedule(new IncrementedMoveArmCommand(lift, deposit, -1)));

        //Score
        gamepadEx2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> schedule(new ScoreCommand(lift, deposit)));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.B)
                .whenReleased(() -> schedule(new SequentialCommandGroup (new UnscoreCommand(lift, deposit),
                                                                         new WaitCommand(Globals.AUTO_RESET_DELAY),
                                                                         new ResetArmCommand(lift, deposit))));

        //Emergency Eject
        gamepadEx2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> schedule(new EjectCommand(lift, deposit)));

        //Climb
        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> schedule(new ClimbUpCommand(lift, deposit)));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(() -> schedule(new DisableAllCommand(robot)));

        //Drone Open
        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> robot.droneLatch.setPosition(Globals.DRONE_OPEN));

    }

    @Override
    public void run() {
        super.run();

        if (timer == null) {
            timer = new ElapsedTime();
            robot.startIMUThread(this);
            fw = new SlewRateLimiter(fw_r);
            str = new SlewRateLimiter(str_r);
        }

        robot.read(drivetrain, lift);

        if (gamepad1.right_stick_button && Globals.USING_IMU)
            SwerveDrivetrain.imuOffset = robot.getAngle() + 0;

        if (gamepad1.right_stick_x < -0.35) {
            lock_robot_heading = true;
            targetHeading = Math.PI/2 + SwerveDrivetrain.imuOffset;
        }

        if (gamepad1.right_stick_x > 0.35) {
            lock_robot_heading = true;
            targetHeading = -Math.PI/2 + SwerveDrivetrain.imuOffset;
        }

        if (gamepad1.right_stick_y < - 0.35) {
            lock_robot_heading = true;
            targetHeading = 0 + SwerveDrivetrain.imuOffset;
        }

        if (gamepad1.right_stick_y > 0.35) {
            lock_robot_heading = true;
            targetHeading = Math.PI + SwerveDrivetrain.imuOffset;
        }

        double turn = gamepad1.right_trigger - gamepad1.left_trigger;
        if (Math.abs(turn) > 0.002) {
            lock_robot_heading = false;
        }

        double error = normalizeRadians(normalizeRadians(targetHeading) - normalizeRadians(robot.getAngle()));
        double headingCorrection = -hController.calculate(0, error) * 12.4 / robot.getVoltage();

        if (Math.abs(headingCorrection) < 0.01) {
            headingCorrection = 0;
        }

        SwerveDrivetrain.maintainHeading =
                (Math.abs(gamepad1.left_stick_x) < 0.002 &&
                        Math.abs(gamepad1.left_stick_y) < 0.002 &&
                        Math.abs(turn) < 0.002) &&
                        Math.abs(headingCorrection) < 0.02;


        double rotationAmount = (Globals.USING_IMU) ? robot.getAngle() - SwerveDrivetrain.imuOffset : 0;
        Pose drive = new Pose(
                new Point(joystickScalar(gamepad1.left_stick_y, 0.001),
                        joystickScalar(gamepad1.left_stick_x, 0.001)).rotate(rotationAmount),
                lock_robot_heading ? headingCorrection :
                        joystickScalar(turn, 0.01)
        );

        drive = new Pose(
                fw.calculate(drive.x),
                str.calculate(drive.y),
                drive.heading
        );

        robot.loop(drive, drivetrain, lift);
        robot.write(drivetrain, lift);
        localizer.periodic();

        double loop = System.nanoTime();
        Pose currentPose = localizer.getPos();

        telemetry.addData("hz: ", 1000000000 / (loop - loopTime));
        /*telemetry.addData("target: ", lift.getTargetPos());
        telemetry.addData("leftArm: ", robot.leftArmEncoder.getPosition());
        telemetry.addData("rightArm: ", robot.rightArmEncoder.getPosition());
        telemetry.addData("parallel: ", robot.parallelPod.getPosition());
        telemetry.addData("perpendicular: ", robot.perpindicularPod.getPosition());
        telemetry.addData("poseX", currentPose.x);
        telemetry.addData("poseY", currentPose.y);*/

        loopTime = loop;
        telemetry.update();

        robot.clearBulkCache();
    }

    private double joystickScalar(double num, double min) {
        return joystickScalar(num, min, 0.66, 4);
    }

    private double joystickScalar(double n, double m, double l, double a) {
        return Math.signum(n) * m
                + (1 - m) *
                (Math.abs(n) > l ?
                        Math.pow(Math.abs(n), Math.log(l / a) / Math.log(l)) * Math.signum(n) :
                        n / a);
    }
}
