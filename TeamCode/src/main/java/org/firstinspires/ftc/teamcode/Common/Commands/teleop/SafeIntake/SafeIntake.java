package org.firstinspires.ftc.teamcode.Common.Commands.teleop.SafeIntake;

public class SafeIntake {

    public static SafeIntakeState sIntakeState = SafeIntakeState.OFF;
    public enum SafeIntakeState {
        OFF,
        ON,
        TRANSFER
    }

    public void update(SafeIntakeState state) {

        sIntakeState = state;

        switch (state) {
        case OFF:
            break;
        case ON:
            break;
        case TRANSFER:
            break;
        }

    }

}