package frc.robot.subsystems.superstructure.pivot;

import org.littletonrobotics.junction.AutoLog;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        LoggedEncodedMotor leftMotor = new LoggedEncodedMotor();
        LoggedEncodedMotor rightMotor = new LoggedEncodedMotor();
    }

    public default void setPivotVoltage (double voltage) {    }

    public default void setPivotPosition (double position) {    }
}
