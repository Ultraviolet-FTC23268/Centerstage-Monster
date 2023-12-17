package org.firstinspires.ftc.teamcode.Common.Commands.teleop;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.abobot.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;

public class UnscoreCommand extends SequentialCommandGroup {
    public UnscoreCommand(LiftSubsystem lift, DepositSubsystem deposit) {

        if(lift.isUp)
            addCommands(
                new DepositCommand(deposit, DepositSubsystem.DepositState.RETRACTED),
                new WaitCommand(Globals.RESET_DELAY),
                new DepositCommand(deposit, DepositSubsystem.DepositState.READY)
            );

    }
}
