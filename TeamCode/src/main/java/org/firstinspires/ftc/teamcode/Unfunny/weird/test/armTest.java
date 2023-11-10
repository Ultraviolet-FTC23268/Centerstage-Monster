package org.firstinspires.ftc.teamcode.Unfunny.weird.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Funny.Drivetrain.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.Funny.Drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.Funny.Utility.Globals;
import org.firstinspires.ftc.teamcode.Funny.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Funny.commandbase.auton.PositionCommand;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

@Config
//@Disabled
@TeleOp(name = "armTest")
public class armTest extends CommandOpMode {
    private ElapsedTime timer;
    private double loopTime = 0;

    GamepadEx gamepadEx, gamepadEx2;

    private final RobotHardware robot = RobotHardware.getInstance();
    private SwerveDrivetrain drivetrain;

    TwoWheelLocalizer localizer;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Globals.AUTO = false;
        Globals.USE_WHEEL_FEEDFORWARD = true;
        Globals.USING_IMU = true;

        robot.init(hardwareMap, telemetry);
        drivetrain = new SwerveDrivetrain(robot);
        localizer = new TwoWheelLocalizer(robot);

        robot.enabled = true;

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot.armLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.armRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();*/

        drivetrain.frontLeftModule.setTargetRotation(0);
        drivetrain.frontRightModule.setTargetRotation(0);
        drivetrain.backRightModule.setTargetRotation(0);
        drivetrain.backLeftModule.setTargetRotation(0);

        while (!this.isStarted()) {
            drivetrain.read();
            drivetrain.updateModules();
            robot.clearBulkCache();

            telemetry.addData("t", drivetrain.frontLeftModule.getTargetRotation());
            telemetry.addData("c", drivetrain.frontLeftModule.getModuleRotation());
            telemetry.update();
        }
    }

    @Override
    public void run() {
        super.run();
        if (timer == null) {
            timer = new ElapsedTime();
            robot.reset();
        }

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> robot.armLeftMotor.setPower(1));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> robot.armRightMotor.setPower(1));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> robot.armLeftMotor.setPower(-1));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> robot.armRightMotor.setPower(-1));

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> robot.armLeftMotor.setPower(0));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> robot.armRightMotor.setPower(0));

        drivetrain.read();
        drivetrain.updateModules();
        drivetrain.write();
        localizer.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("x", localizer.getPos().x);
        telemetry.addData("y", localizer.getPos().y);
        telemetry.addData("h", localizer.getPos().heading);
        telemetry.addData("xP", PositionCommand.xP);
        telemetry.addData("xD", PositionCommand.xD);
        telemetry.addData("t", drivetrain.frontLeftModule.getTargetRotation());
        telemetry.addData("c", drivetrain.frontLeftModule.getModuleRotation());
        loopTime = loop;
        telemetry.update();

        robot.clearBulkCache();
    }
}
