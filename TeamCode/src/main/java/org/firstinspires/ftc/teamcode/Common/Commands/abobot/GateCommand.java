package org.firstinspires.ftc.teamcode.Common.Commands.abobot;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;

public class GateCommand extends InstantCommand {
    public GateCommand(DepositSubsystem deposit, DepositSubsystem.GateState state) {
        super(
                () -> deposit.update(state)
        );
    }
}
