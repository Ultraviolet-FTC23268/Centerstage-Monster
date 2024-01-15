package org.firstinspires.ftc.teamcode.Common.Commands.teleop;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.abobot.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.GateCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;

public class ScoreCommand extends SequentialCommandGroup {
    public ScoreCommand(LiftSubsystem lift, DepositSubsystem deposit) {

        if(lift.isUp) {
            addCommands(
                    new GateCommand(deposit, DepositSubsystem.GateState.OPEN),
                    new WaitCommand(Globals.SCORE_DROP_DELAY)
            );
        }

    }
}
