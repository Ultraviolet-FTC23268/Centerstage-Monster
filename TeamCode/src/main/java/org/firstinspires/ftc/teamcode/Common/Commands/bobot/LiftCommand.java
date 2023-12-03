package org.firstinspires.ftc.teamcode.Common.Commands.bobot;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;

public class LiftCommand extends InstantCommand {
    public LiftCommand(LiftSubsystem lift, LiftSubsystem.LiftStateReel state) {
        super(
                () -> lift.update(state)
        );
    }
}
