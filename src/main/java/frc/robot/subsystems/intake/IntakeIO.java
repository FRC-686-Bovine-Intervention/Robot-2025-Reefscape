package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.util.loggerUtil.inputs.LoggedFaults;
import frc.util.loggerUtil.inputs.LoggedMotor;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        boolean motorConnected = false;
        LoggedMotor motor = new LoggedMotor();
        LoggedFaults motorFaults = new LoggedFaults();

        boolean coralSensor = false;
        boolean algaeSensor = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
    
    public default void setVolts(double volts) {}
    
    public default void clearMotorStickyFaults(long bitmask) {}
}
