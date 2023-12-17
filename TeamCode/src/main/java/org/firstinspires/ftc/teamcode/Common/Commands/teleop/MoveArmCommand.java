package org.firstinspires.ftc.teamcode.Common.Commands.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.abobot.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;

import org.firstinspires.ftc.teamcode.Common.Utility.Globals;

public class MoveArmCommand extends SequentialCommandGroup {
    public MoveArmCommand(LiftSubsystem lift, DepositSubsystem deposit, LiftSubsystem.LiftStateReel state) {

        super(
                new LiftCommand(lift, state),
                new WaitCommand(Globals.FLIP_OUT_DELAY),
                new DepositCommand(deposit, DepositSubsystem.DepositState.READY)
        );

    }
}
