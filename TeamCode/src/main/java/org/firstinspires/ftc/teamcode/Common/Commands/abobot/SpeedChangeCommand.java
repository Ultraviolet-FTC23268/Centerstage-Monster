package org.firstinspires.ftc.teamcode.Common.Commands.abobot;

import static org.firstinspires.ftc.teamcode.Common.Drivetrain.swerve.SwerveModule.MAX_MOTOR;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Common.Drivetrain.swerve.SwerveModule;

public class SpeedChangeCommand extends InstantCommand {
    public SpeedChangeCommand(double power) {

        MAX_MOTOR = power;

    }
}
