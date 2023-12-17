package org.firstinspires.ftc.teamcode.Common.Commands.abobot;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;

public class DepositCommand extends InstantCommand {
    public DepositCommand(DepositSubsystem deposit, DepositSubsystem.DepositState state) {
        super(
                () -> deposit.update(state)
        );
    }
}
