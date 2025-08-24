package frc.robot.subsystems.superstructure.elevator;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;
import frc.util.loggerUtil.inputs.LoggedEncoder;
import frc.util.loggerUtil.inputs.LoggedFaults;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        LoggedEncoder encoder = new LoggedEncoder();
        LoggedEncodedMotor motor = new LoggedEncodedMotor();
        LoggedFaults encoderFaults = new LoggedFaults();
        LoggedFaults motorFaults = new LoggedFaults();
    } 

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setVolts(double volts) {}

    public default void setPosition(double positionRads, double velocityRadsPerSec, double feedforwardVolts) {}
    
    public default void stop(Optional<NeutralMode> neutralMode) {}

    public default void configPID(PIDConstants pidConstants) {}

    public default void clearMotorStickyFaults(long bitmask) {}
    public default void clearEncoderStickyFaults(long bitmask) {}
}