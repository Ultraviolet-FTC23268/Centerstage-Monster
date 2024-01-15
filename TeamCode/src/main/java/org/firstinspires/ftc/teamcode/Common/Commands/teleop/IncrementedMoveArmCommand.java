package org.firstinspires.ftc.teamcode.Common.Commands.teleop;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.abobot.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.GateCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.IncrementLiftCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;

public class IncrementedMoveArmCommand extends SequentialCommandGroup {
    public IncrementedMoveArmCommand(LiftSubsystem lift, DepositSubsystem deposit, int amount) {

        super(
                new GateCommand(deposit, DepositSubsystem.GateState.CLOSED),
                new IncrementLiftCommand(lift, amount),
                new WaitCommand(Globals.FLIP_OUT_DELAY),
                new DepositCommand(deposit, DepositSubsystem.DepositState.DEPOSIT)
        );

    }
}
