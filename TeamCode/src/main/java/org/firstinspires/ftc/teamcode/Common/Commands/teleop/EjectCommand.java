package org.firstinspires.ftc.teamcode.Common.Commands.teleop;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.abobot.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;

public class EjectCommand extends SequentialCommandGroup {
    public EjectCommand(LiftSubsystem lift, DepositSubsystem deposit) {

        if(!lift.isUp)
            addCommands(
                new LiftCommand(lift, LiftSubsystem.LiftStateReel.ROW3),
                new WaitCommand(Globals.FLIP_OUT_DELAY),
                new DepositCommand(deposit, DepositSubsystem.DepositState.DEPOSIT1),
                new WaitCommand(Globals.EJECT_DELAY),
                new ResetArmCommand(lift, deposit)
            );

    }
}
