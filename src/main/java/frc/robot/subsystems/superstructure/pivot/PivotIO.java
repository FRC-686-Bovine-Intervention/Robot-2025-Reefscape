package frc.robot.subsystems.superstructure.pivot;

import frc.util.loggerUtil.inputs.LoggedMotor;

public interface PivotIO {
    public default void setPivotVoltage (double voltage) {    }

    public default void setPivotPosition (double position) {    }
}
