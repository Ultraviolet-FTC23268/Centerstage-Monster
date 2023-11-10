package org.firstinspires.ftc.teamcode.Funny.Drivetrain.localizer;

import org.firstinspires.ftc.teamcode.Funny.Drivetrain.geometry.Pose;

public interface Localizer {

    void periodic();

    Pose getPos();

    void setPos(Pose pose);
}
