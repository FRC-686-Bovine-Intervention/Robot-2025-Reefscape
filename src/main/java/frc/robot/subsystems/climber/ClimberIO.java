package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;
import frc.util.loggerUtil.inputs.LoggedFaults;

public interface ClimberIO {
    @AutoLog
    public class ClimberIOInputs {
        boolean motorConnected = false;
        public LoggedEncodedMotor motor = new LoggedEncodedMotor();
        public LoggedFaults motorFaults = new LoggedFaults();

        public boolean sensor = false;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setVoltage(Measure<VoltageUnit> voltage, boolean brakeMode) {}

    public default void setRatchetServoAngle(Measure<AngleUnit> angle) {}

    public default void setNonClimbingAngle(Measure<AngleUnit> angle) {}

    public default void setClimbingAngle(Measure<AngleUnit> angle) {}

    public default void clearMotorStickyFaults(long bitmask) {}
}
