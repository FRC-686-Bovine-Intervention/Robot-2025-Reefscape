package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.util.loggerUtil.inputs.LoggedMotor;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        LoggedMotor leftMotor = new LoggedMotor();
        LoggedMotor rightMotor = new LoggedMotor();
    } 

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setVoltage(Measure<VoltageUnit> voltage) {}

    public default void setPosition(Measure<DistanceUnit> dist) {}

    public default void stop() {}
}