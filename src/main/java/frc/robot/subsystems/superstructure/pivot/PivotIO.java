package frc.robot.subsystems.superstructure.pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;
import frc.util.loggerUtil.inputs.LoggedEncoder;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        LoggedEncoder encoder = new LoggedEncoder();
        LoggedEncodedMotor leftMotor = new LoggedEncodedMotor();
        LoggedEncodedMotor rightMotor = new LoggedEncodedMotor();
    }

    public default void updateInputs(PivotIOInputs inputs) {}

    public default void setVoltage(Measure<VoltageUnit> voltage) {}

    public default void setPosition(Measure<AngleUnit> position) {}

    public default void setFeedForward(Measure<VoltageUnit> feedForward) {}

    public default void setCoastMode() {}

    public default void stop() {}
}
