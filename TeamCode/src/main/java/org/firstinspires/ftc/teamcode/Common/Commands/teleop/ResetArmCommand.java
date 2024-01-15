package org.firstinspires.ftc.teamcode.Common.Commands.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.abobot.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.GateCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;

public class ResetArmCommand extends SequentialCommandGroup {
    public ResetArmCommand(LiftSubsystem lift, DepositSubsystem deposit) {

        if(lift.isUp)
            addCommands(
                    new InstantCommand( () -> lift.setTargetPos(lift.getTargetPos() + Globals.LIFT_RESET_OFFSET)),
                    new DepositCommand(deposit, DepositSubsystem.DepositState.INTERMEDIATE),
                    new WaitCommand(Globals.FLIP_AROUND_DELAY),
                    new GateCommand(deposit, DepositSubsystem.GateState.CLOSED),
                    new InstantCommand( () -> lift.setTargetPos(Globals.SWING_IN_POS))
                            .alongWith(new WaitUntilCommand(lift::isWithinTolerance)),
                    new WaitCommand(Globals.LIFT_SETTLE_DELAY),
                    new DepositCommand(deposit, DepositSubsystem.DepositState.INTAKE),
                    new LiftCommand(lift, LiftSubsystem.LiftStateReel.DOWN)
                            .alongWith(new WaitUntilCommand(lift::isWithinTolerance)),
                    new GateCommand(deposit, DepositSubsystem.GateState.OPEN)

            );

    }
}
