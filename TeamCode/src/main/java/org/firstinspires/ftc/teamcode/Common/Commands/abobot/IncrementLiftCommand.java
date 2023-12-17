package org.firstinspires.ftc.teamcode.Common.Commands.abobot;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;

public class IncrementLiftCommand extends InstantCommand {
    public IncrementLiftCommand(LiftSubsystem lift, int amount) {
        super(
                () -> lift.changeRow(amount)
        );
    }
}
