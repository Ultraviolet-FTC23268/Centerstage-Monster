package org.firstinspires.ftc.teamcode.Funny.Drivetrain.Fancy.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Funny.Drivetrain.swerve.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.Funny.Drivetrain.geometry.Pose;
import org.firstinspires.ftc.teamcode.Funny.Drivetrain.localizer.Localizer;
import org.firstinspires.ftc.teamcode.Funny.Drivetrain.localizer.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.Funny.Utility.Globals;
import org.firstinspires.ftc.teamcode.Funny.Utility.RobotHardware;

@Config
@TeleOp(name = "gvftest")
public class GVFTester extends CommandOpMode {

    private SwerveDrivetrain drivetrain;
    private final RobotHardware robot = RobotHardware.getInstance();
    GamepadEx gamepadEx;
    Localizer localizer;

    private double loopTime;

    boolean started = false;

    HermitePath path1 = new HermitePath()
            .addPose(0.0, 0.0, new Vector2D(250.0, 0.0))
            .addPose(-60.0, 0.0, new Vector2D(500.0, 0.0))
            .addPose(-75.0, -15.0, new Vector2D(0.0, 500.0))
            .construct();

    HermitePath path2 = new HermitePath()
            .addPose(-75.0, -15.0, new Vector2D(0.0, 250.0))
            .addPose(-60.0, 0.0, new Vector2D(0.0, 500.0))
            .addPose(0.0, 0.0, new Vector2D(500.0, 0.0))
            .construct();

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Globals.AUTO = true;
        Globals.USING_IMU = true;
        Globals.USE_WHEEL_FEEDFORWARD = false;

        robot.init(hardwareMap, telemetry);
        drivetrain = new SwerveDrivetrain(robot);
        gamepadEx = new GamepadEx(gamepad1);
        localizer = new TwoWheelLocalizer(robot);


        robot.enabled = true;

        /*PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();*/

        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> schedule(new GVFCommand(drivetrain, localizer, path1, 12)));
        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> schedule(new GVFCommand(drivetrain, localizer, path2, 12)));

    }

    @Override
    public void run() {
        super.run();
        if (!started) {
            started = true;

            robot.startIMUThread(this);
        }

        robot.read(drivetrain);

        CommandScheduler.getInstance().run();
        robot.loop(null, drivetrain);
        localizer.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        telemetry.addData("GVF :", GVFController.gvf22);
        telemetry.addData("GVF pow:", GVFController.powers);
        loopTime = loop;
        telemetry.update();
        robot.write(drivetrain);
        robot.clearBulkCache();
    }
}
