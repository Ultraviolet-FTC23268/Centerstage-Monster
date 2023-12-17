package org.firstinspires.ftc.teamcode.Common.Commands.teleop;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.abobot.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

public class DisableAllCommand extends SequentialCommandGroup {
    public DisableAllCommand(RobotHardware robot) {

        robot.frontRightServo.setPower(0);
        robot.frontRightServo.setPower(0);
        robot.backRightServo.setPower(0);
        robot.backLeftServo.setPower(0);
        robot.leftElbow.getController().pwmDisable();
        robot.rightElbow.getController().pwmDisable();

        robot.leftElbow.getController().close();
        robot.rightElbow.getController().close();

        robot.droneLatch.getController().pwmDisable();

        robot.droneLatch.getController().close();

    }
}
