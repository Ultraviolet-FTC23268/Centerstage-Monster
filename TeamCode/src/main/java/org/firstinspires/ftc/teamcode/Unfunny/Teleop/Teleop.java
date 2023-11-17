package org.firstinspires.ftc.teamcode.Unfunny.Teleop;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Funny.Drivetrain.swerve.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.Funny.Drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.Funny.Drivetrain.geometry.Point;
import org.firstinspires.ftc.teamcode.Funny.Drivetrain.geometry.Pose;
import org.firstinspires.ftc.teamcode.Funny.Drivetrain.localizer.Localizer;
import org.firstinspires.ftc.teamcode.Funny.Drivetrain.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.Funny.Utility.Globals;
import org.firstinspires.ftc.teamcode.Funny.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Funny.commandbase.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Funny.commandbase.bobot.IntakeCommand;
import org.firstinspires.ftc.teamcode.Funny.commandbase.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Funny.commandbase.bobot.LiftCommand;

//@Photon(maximumParallelCommands = 8)
@Config
@TeleOp(name = "\uD83D\uDCB2\uD83D\uDCB8\uD83E\uDD11")
public class Teleop extends CommandOpMode {
    private ElapsedTime timer;
    private double loopTime = 0;

    public static double position;

    private boolean pHeadingLock = true;
    private double targetHeading;

    private final RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;
    private IntakeSubsystem intake;
    private LiftSubsystem lift;

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
        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        localizer = new TwoWheelLocalizer(robot);

        robot.enabled = true;

        //robot.LEDcontroller.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> schedule(new IntakeCommand(intake, IntakeSubsystem.IntakeState.IDLE)));
        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> schedule(new IntakeCommand(intake, IntakeSubsystem.IntakeState.INWARDS)));
        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> schedule(new IntakeCommand(intake, IntakeSubsystem.IntakeState.OUTWARDS)));
        gamepadEx.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> schedule(new IntakeCommand(intake, IntakeSubsystem.IntakeState.OFF)));

        /*gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> robot.LEDcontroller.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> robot.LEDcontroller.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> robot.LEDcontroller.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> robot.LEDcontroller.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK));

       *gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> schedule(new LiftCommand(lift, LiftSubsystem.LiftState.UPWARDS)));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> schedule(new LiftCommand(lift, LiftSubsystem.LiftState.OFF)));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> schedule(new LiftCommand(lift, LiftSubsystem.LiftState.DOWNWARDS)));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> robot.droneMotor.setPower(1));
        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> robot.droneMotor.setPower(0));*/

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

        robot.read(drivetrain);

        if (gamepad1.right_stick_button && Globals.USING_IMU)
            SwerveDrivetrain.imuOffset = robot.getAngle() + 0;

        if (gamepad1.right_stick_button && Globals.USING_IMU)
            robot.imu.resetYaw();

        if (gamepad1.right_stick_x < -0.35) {
            lock_robot_heading = true;
            targetHeading = Math.PI/2 + SwerveDrivetrain.imuOffset;
        }

        if (gamepad1.right_stick_x > 0.35) {
            lock_robot_heading = true;
            targetHeading = Math.PI/2 - SwerveDrivetrain.imuOffset;
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

        robot.loop(drive, drivetrain);
        robot.write(drivetrain);
        localizer.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz: ", 1000000000 / (loop - loopTime));
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
