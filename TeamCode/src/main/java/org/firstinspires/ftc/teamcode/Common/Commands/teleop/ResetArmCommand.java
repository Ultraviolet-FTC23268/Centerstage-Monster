package org.firstinspires.ftc.teamcode.Common.Commands.teleop;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.abobot.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;

public class ResetArmCommand extends SequentialCommandGroup {
    public ResetArmCommand(LiftSubsystem lift, DepositSubsystem deposit) {

        if(lift.isUp)
            addCommands(
                    new DepositCommand(deposit, DepositSubsystem.DepositState.INTERMEDIATE)
                            .alongWith(new LiftCommand(lift, LiftSubsystem.LiftStateReel.ROW1)),
                    new WaitCommand(Globals.LIFT_DELAY),
                    new DepositCommand(deposit, DepositSubsystem.DepositState.INTERMEDIATE2),
                    new WaitCommand(Globals.FLIP_IN_DELAY),
                    new LiftCommand(lift, LiftSubsystem.LiftStateReel.DOWN),
                    new WaitCommand(Globals.FULL_READY_DELAY),
                    new DepositCommand(deposit, DepositSubsystem.DepositState.INTAKE)
            );

    }
}
