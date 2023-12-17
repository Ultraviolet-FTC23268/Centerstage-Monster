package org.firstinspires.ftc.teamcode.Common.Commands.abobot;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;

public class GateCommand extends InstantCommand {
    public GateCommand(IntakeSubsystem intake, IntakeSubsystem.GateState state) {
        super(
                () -> intake.update(state)
        );
    }
}
