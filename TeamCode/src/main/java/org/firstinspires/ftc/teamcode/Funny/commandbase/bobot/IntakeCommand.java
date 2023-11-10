package org.firstinspires.ftc.teamcode.Funny.commandbase.bobot;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Funny.commandbase.subsystems.IntakeSubsystem;

public class IntakeCommand extends InstantCommand {
    public IntakeCommand(IntakeSubsystem intake, IntakeSubsystem.IntakeState state) {
        super(
                () -> intake.update(state)
        );
    }
}
