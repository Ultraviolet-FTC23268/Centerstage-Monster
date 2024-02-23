package org.firstinspires.ftc.teamcode.Common.Commands.abobot;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

public class LiftCommand extends InstantCommand {
    public LiftCommand(LiftSubsystem.LiftStateReel state) {
        super(
                () ->  RobotHardware.getInstance().lift.update(state)
        );
    }
}
