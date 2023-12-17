package org.firstinspires.ftc.teamcode.Common.Commands.teleop.SafeIntake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.abobot.GateCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.IntakeCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.teleop.SafeIntake.SafeIntake;
import org.firstinspires.ftc.teamcode.Common.Subsystems.IntakeSubsystem;

public class SafeIntakeCommand extends SequentialCommandGroup {
    public SafeIntakeCommand(IntakeSubsystem intake) {

        if(SafeIntake.sIntakeState == SafeIntake.SafeIntakeState.OFF) {

            addCommands(
                new GateCommand(intake, IntakeSubsystem.GateState.CLOSED),
                new WaitCommand(50),
                new IntakeCommand(intake, IntakeSubsystem.IntakeState.MAX)
            );


        }

        if(true) {

            addCommands(
                    new GateCommand(intake, IntakeSubsystem.GateState.CLOSED),
                    new WaitCommand(50),
                    new IntakeCommand(intake, IntakeSubsystem.IntakeState.MAX)
            );

        }
                //new InstantCommand(() -> intake.setTargetPosition(0)),
    }
}
