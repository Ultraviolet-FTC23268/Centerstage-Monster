package org.firstinspires.ftc.teamcode.Common.Commands.abobot;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

public class GateCommand extends InstantCommand {
    public GateCommand(DepositSubsystem.GateState state) {
        super(
                () ->  RobotHardware.getInstance().deposit.update(state)
        );
    }
}
