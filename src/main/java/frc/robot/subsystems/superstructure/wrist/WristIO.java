package frc.robot.subsystems.superstructure.wrist;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;
import frc.util.loggerUtil.inputs.LoggedEncoder;
import frc.util.loggerUtil.inputs.LoggedFaults;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        LoggedEncoder encoder = new LoggedEncoder();
        LoggedEncodedMotor motor = new LoggedEncodedMotor();
        LoggedFaults encoderFaults = new LoggedFaults();
        LoggedFaults motorFaults = new LoggedFaults();
    }
    
    public default void updateInputs(WristIOInputs inputs) {}

    public default void setVoltage(Measure<VoltageUnit> voltage) {}
    
    public default void setPosition(Measure<AngleUnit> position, Measure<AngularVelocityUnit> velocity, Measure<VoltageUnit> feedforward) {}

    public default void configPID(PIDConstants pidConstants) {}

    public default void stop(Optional<NeutralMode> neutralMode) {}
}
