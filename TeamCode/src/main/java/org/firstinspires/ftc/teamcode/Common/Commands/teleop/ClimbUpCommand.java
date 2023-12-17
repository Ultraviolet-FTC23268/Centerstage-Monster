package org.firstinspires.ftc.teamcode.Common.Commands.teleop;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.abobot.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;

public class ClimbUpCommand extends SequentialCommandGroup {
    public ClimbUpCommand(LiftSubsystem lift, DepositSubsystem deposit) {

        super(
                new LiftCommand(lift, LiftSubsystem.LiftStateReel.MAX),
                new WaitCommand(Globals.FLIP_OUT_DELAY),
                new DepositCommand(deposit, DepositSubsystem.DepositState.HANG)
        );

    }
}
