package org.firstinspires.ftc.teamcode.Common.Commands.abobot;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

public class IncrementLiftCommand extends InstantCommand {
    public IncrementLiftCommand(int amount) {
        super(
                () ->  RobotHardware.getInstance().lift.changeRow(amount)
        );
    }
}
