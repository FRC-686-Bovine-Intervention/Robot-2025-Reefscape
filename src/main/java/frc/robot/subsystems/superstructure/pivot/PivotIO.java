package frc.robot.subsystems.superstructure.pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        LoggedEncodedMotor leftMotor = new LoggedEncodedMotor();
        LoggedEncodedMotor rightMotor = new LoggedEncodedMotor();
    }

    public default void setVoltage(double voltage) {}

    public default void setPosition(Angle position) {}

    public default void stop() {}
}
