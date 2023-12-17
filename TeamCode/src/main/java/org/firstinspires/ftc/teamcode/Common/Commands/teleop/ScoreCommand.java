package org.firstinspires.ftc.teamcode.Common.Commands.teleop;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.abobot.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;

public class ScoreCommand extends SequentialCommandGroup {
    public ScoreCommand(LiftSubsystem lift, DepositSubsystem deposit) {

        if(lift.isUp) {
            if(lift.liftStateReel == LiftSubsystem.LiftStateReel.ROW1 || lift.liftStateReel == LiftSubsystem.LiftStateReel.ROW2 || lift.liftStateReel == LiftSubsystem.LiftStateReel.ROW3 )
                addCommands(
                    new DepositCommand(deposit, DepositSubsystem.DepositState.DEPOSIT1),
                    new WaitCommand(Globals.RETRACTION_DELAY)
                );
            else if(lift.liftStateReel == LiftSubsystem.LiftStateReel.ROW4 || lift.liftStateReel == LiftSubsystem.LiftStateReel.ROW5)
                addCommands(
                        new DepositCommand(deposit, DepositSubsystem.DepositState.DEPOSIT2),
                        new WaitCommand(Globals.RETRACTION_DELAY)
                );
            else
                addCommands(
                        new DepositCommand(deposit, DepositSubsystem.DepositState.DEPOSIT3),
                        new WaitCommand(Globals.RETRACTION_DELAY)
                );
        }

    }
}
