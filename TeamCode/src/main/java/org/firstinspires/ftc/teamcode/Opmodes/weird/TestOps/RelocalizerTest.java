package org.firstinspires.ftc.teamcode.Opmodes.weird.TestOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Common.Commands.auton.RelocalizeCommand;
import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.Pose;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;
import org.firstinspires.ftc.teamcode.Common.Utility.MathUtils;
import org.firstinspires.ftc.teamcode.Common.Vision.Location;

@Autonomous(name = "RelocalizerTest")
//@Disabled
public class RelocalizerTest extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private double loopTime = 0.0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.AUTO = true;
        Globals.ALLIANCE = Location.BLUE;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        robot.init(hardwareMap, telemetry);

        robot.read();

        robot.startIMUThread(this);
        robot.localizer.setPos(new Pose(-62,  16, 0));

        while (!isStarted()) {
            telemetry.addLine("auto in init");
            telemetry.update();
        }

        robot.startCamera();
    }

    @Override
    public void run() {
        if (isStopRequested()) robot.closeCamera();

        robot.read();
        Pose drive = new Pose(gamepad1.left_stick_x, -gamepad1.left_stick_y, MathUtils.joystickScalar(-gamepad1.left_trigger + gamepad1.right_trigger, 0.01));
        robot.loop(drive);

        super.run();
        robot.localizer.periodic();

        Pose currentPose = robot.localizer.getPos();
        Pose globalTagPosition = robot.getAprilTagPosition();

        if (globalTagPosition == null) globalTagPosition = new Pose();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.addData("tag", globalTagPosition.toString());
        telemetry.addData("three", currentPose.toString());

        telemetry.update();

        if (gamepad1.a) {
            CommandScheduler.getInstance().schedule(new RelocalizeCommand());
////            localizer.setPose(globalTagPosition);
       }
//
        robot.write();
        robot.clearBulkCache();
    }

}