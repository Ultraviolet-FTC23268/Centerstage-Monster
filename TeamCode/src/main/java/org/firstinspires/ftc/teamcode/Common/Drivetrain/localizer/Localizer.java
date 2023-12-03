package org.firstinspires.ftc.teamcode.Common.Drivetrain.localizer;

import org.firstinspires.ftc.teamcode.Common.Drivetrain.geometry.Pose;

public interface Localizer {

    void periodic();

    Pose getPos();

    void setPos(Pose pose);
}
