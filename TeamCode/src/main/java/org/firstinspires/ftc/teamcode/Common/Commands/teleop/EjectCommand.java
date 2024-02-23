package org.firstinspires.ftc.teamcode.Common.Commands.teleop;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Common.Commands.abobot.DepositCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.GateCommand;
import org.firstinspires.ftc.teamcode.Common.Commands.abobot.LiftCommand;
import org.firstinspires.ftc.teamcode.Common.Subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.Common.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

public class EjectCommand extends SequentialCommandGroup {
    public EjectCommand() {

        if(!RobotHardware.getInstance().lift.isUp)
            addCommands(
                    new LiftCommand(LiftSubsystem.LiftStateReel.ROW1),
                    new WaitCommand(Globals.FLIP_OUT_DELAY),
                    new DepositCommand(DepositSubsystem.DepositState.DEPOSIT),
                    new WaitCommand(Globals.EJECT_DELAY),
                    new GateCommand(DepositSubsystem.GateState.OPEN),
                    new ResetArmCommand()
            );

    }
}
