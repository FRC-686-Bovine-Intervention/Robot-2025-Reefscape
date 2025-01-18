package frc.robot.subsystems.superstructure.wrist;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;
import frc.util.loggerUtil.inputs.LoggedEncoder;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        LoggedEncoder encoder = new LoggedEncoder();
        LoggedEncodedMotor motor = new LoggedEncodedMotor();
    }
    public default void updateInputs (WristIOInputs inputs) {}

    public default void setVoltage(Measure<VoltageUnit> voltage) {}
    public default void setAngle(Measure<AngleUnit> angle) {}
    public default void stop() {}
}
