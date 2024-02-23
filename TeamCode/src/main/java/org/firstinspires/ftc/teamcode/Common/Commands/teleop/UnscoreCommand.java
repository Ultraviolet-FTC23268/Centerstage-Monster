package org.firstinspires.ftc.teamcode.Common.Commands.teleop;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.abobot.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

public class UnscoreCommand extends SequentialCommandGroup {
    public UnscoreCommand() {

        if( RobotHardware.getInstance().lift.isUp)
            addCommands(
                new DepositCommand(DepositSubsystem.DepositState.RETRACTED),
                new WaitCommand(Globals.RESET_DELAY),
                new DepositCommand(DepositSubsystem.DepositState.INTERMEDIATE)
            );

    }
}
