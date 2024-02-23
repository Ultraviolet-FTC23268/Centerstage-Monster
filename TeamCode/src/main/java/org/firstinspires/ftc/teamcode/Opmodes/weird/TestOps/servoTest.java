package org.firstinspires.ftc.teamcode.Opmodes.weird.TestOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@Config
@TeleOp(name = "servoTest")
//@Autonomous(name = "servoTest")
public class servoTest extends LinearOpMode {

    public static double leftPos = 0;
    public static double rightPos = 0;
    public static double gatePos = 0;
    public static double pPos = 0;

    private final RobotHardware robot = RobotHardware.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        while (opModeInInit()) {

            robot.init(hardwareMap, telemetry);

        }

        waitForStart();

        while (opModeIsActive()) {

            //robot.rightElbow.setPosition(rightPos);
            //robot.leftElbow.setPosition(leftPos);
            //robot.gateServo.setPosition(gatePos);
            robot.purpleLatch.setPosition(pPos);

        }
    }
}
