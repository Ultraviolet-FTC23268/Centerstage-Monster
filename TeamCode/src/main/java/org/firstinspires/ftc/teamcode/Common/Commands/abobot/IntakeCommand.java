package org.firstinspires.ftc.teamcode.Common.Commands.abobot;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;

public class IntakeCommand extends InstantCommand {
    public IntakeCommand(IntakeSubsystem intake, IntakeSubsystem.IntakeState state) {
        super(
                () -> intake.update(state)
        );
    }
}
