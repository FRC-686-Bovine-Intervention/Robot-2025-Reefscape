package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.util.loggerUtil.inputs.LoggedMotor;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public LoggedMotor motor = new LoggedMotor();
    }

    public default void updateInputs(ClimberIOInputs inputs) {    }

    public default void setMotorVoltage(Measure<VoltageUnit> voltage) {    }
}
