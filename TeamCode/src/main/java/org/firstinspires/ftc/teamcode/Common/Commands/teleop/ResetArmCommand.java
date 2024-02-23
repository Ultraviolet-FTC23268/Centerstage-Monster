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
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

public class ResetArmCommand extends SequentialCommandGroup {
    public ResetArmCommand() {

        if(RobotHardware.getInstance().lift.isUp)
            addCommands(
                    new InstantCommand( () -> RobotHardware.getInstance().lift.setTargetPos(RobotHardware.getInstance().lift.getTargetPos() + Globals.LIFT_RESET_OFFSET)),
                    new DepositCommand(DepositSubsystem.DepositState.INTERMEDIATE),
                    new WaitCommand(Globals.FLIP_AROUND_DELAY),
                    new GateCommand( DepositSubsystem.GateState.CLOSED),
                    new InstantCommand( () -> RobotHardware.getInstance().lift.setTargetPos(Globals.SWING_IN_POS))
                            .alongWith(new WaitUntilCommand(RobotHardware.getInstance().lift::isWithinTolerance)),
                    new WaitCommand(Globals.LIFT_SETTLE_DELAY),
                    new DepositCommand( DepositSubsystem.DepositState.INTAKE),
                    new LiftCommand( LiftSubsystem.LiftStateReel.DOWN)
                            .alongWith(new WaitUntilCommand(RobotHardware.getInstance().lift::isWithinTolerance)),
                    new GateCommand(DepositSubsystem.GateState.OPEN)

            );

    }
}
