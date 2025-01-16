package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;
import frc.util.loggerUtil.inputs.LoggedEncoder;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        LoggedEncoder encoder = new LoggedEncoder();
        LoggedEncodedMotor leftMotor = new LoggedEncodedMotor();
        LoggedEncodedMotor rightMotor = new LoggedEncodedMotor();
    } 

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setVoltage(Measure<VoltageUnit> voltage) {}

    public default void setLength(Measure<DistanceUnit> length) {}

    public default void stop() {}
}