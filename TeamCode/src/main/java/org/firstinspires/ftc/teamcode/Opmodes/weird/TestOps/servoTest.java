package org.firstinspires.ftc.teamcode.Opmodes.weird.TestOps;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Other.Pipelines.PropPipeline;
import org.firstinspires.ftc.teamcode.Other.Side;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

//@Disabled
@Config
@TeleOp(name = "servoTest")
//@Autonomous(name = "servoTest")
public class servoTest extends LinearOpMode {

    public static double leftPos = 0;
    public static double rightPos = 0;

    private final RobotHardware robot = RobotHardware.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        while (opModeInInit()) {

            robot.init(hardwareMap, telemetry);

        }

        waitForStart();

        while (opModeIsActive()) {

            robot.rightElbow.setPosition(rightPos);
            robot.leftElbow.setPosition(leftPos);

        }
    }
}
