package frc.robot.subsystems.superstructure.elevator;

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

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        LoggedEncoder encoder = new LoggedEncoder();
        LoggedEncodedMotor motor = new LoggedEncodedMotor();
    } 

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setVoltage(Measure<VoltageUnit> voltage) {}

    public default void setPosition(Measure<AngleUnit> position, Measure<AngularVelocityUnit> velocity, Measure<VoltageUnit> feedforward) {}

    public default void configPID(PIDConstants pidConstants) {}

    public default void stop(Optional<NeutralMode> neutralMode) {}
}