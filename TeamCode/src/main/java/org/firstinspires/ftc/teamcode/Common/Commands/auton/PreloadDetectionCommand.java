package org.firstinspires.ftc.teamcode.Common.Commands.auton;

import org.firstinspires.ftc.teamcode.Common.Commands.auton.swervePositionCommand;
import org.firstinspires.ftc.teamcode.Common.Utility.Globals;
import org.firstinspires.ftc.teamcode.Common.Utility.RobotHardware;

public class PreloadDetectionCommand extends swervePositionCommand {
    private boolean flag = false;
    public PreloadDetectionCommand() {
        // default
        //super(ScoringPositions.YELLOW_PIXEL_POSITIONS[Globals.getTargetIndex()].getTargetPose());
        super(null);
    }

    @Override
    public void execute(){
        super.execute();
        if(!flag) {
            // updated
            //super.targetPose = ScoringPositions.YELLOW_PIXEL_POSITIONS[Globals.getTargetIndex()].getTargetPose();
            flag = true;
            System.out.println("HERE123 " + super.targetPose);

        }
        System.out.println("HERE125 " + RobotHardware.getInstance().localizer.getPos());

    }
}
