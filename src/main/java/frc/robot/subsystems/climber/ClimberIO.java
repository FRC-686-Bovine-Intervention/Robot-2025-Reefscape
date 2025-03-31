package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;

public interface ClimberIO {
    
    @AutoLog
    public class ClimberIOInputs {
        public LoggedEncodedMotor motor = new LoggedEncodedMotor();

        public boolean sensor = false;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setVoltage(Measure<VoltageUnit> voltage) {}

    public default void setRatchetServoAngle(Measure<AngleUnit> angle) {}

    public default void setAngle(Measure<AngleUnit> angle) {}
}
