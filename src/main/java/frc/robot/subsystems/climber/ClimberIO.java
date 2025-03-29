package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;

public interface ClimberIO {
    
    @AutoLog
    public class ClimberIOInputs {
        public LoggedEncodedMotor motor = new LoggedEncodedMotor();
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setCoastVoltage(Measure<VoltageUnit> voltage) {}

    public default void setBrakeVoltage(Measure<VoltageUnit> voltage) {}
}
