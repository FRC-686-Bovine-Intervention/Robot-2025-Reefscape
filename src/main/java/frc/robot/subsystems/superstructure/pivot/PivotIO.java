package frc.robot.subsystems.superstructure.pivot;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;
import frc.util.loggerUtil.inputs.LoggedEncoder;
import frc.util.loggerUtil.inputs.LoggedFaults;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        LoggedEncoder encoder = new LoggedEncoder();
        LoggedEncodedMotor leftMotor = new LoggedEncodedMotor();
        LoggedEncodedMotor rightMotor = new LoggedEncodedMotor();
        LoggedFaults leftMotorFaults = new LoggedFaults();
        LoggedFaults rightMotorFaults = new LoggedFaults();
        LoggedFaults encoderFaults = new LoggedFaults();
    }

    public default void updateInputs(PivotIOInputs inputs) {}

    public default void setVolts(double volts) {}

    public default void setPosition(double positionRads, double velocityRadsPerSec, double feedforwardVolts) {}
    
    public default void stop(Optional<NeutralMode> neutralMode) {}

    public default void configPID(PIDConstants pidConstants) {}

    public default void clearLeftMotorStickyFaults(long bitmask) {}
    public default void clearRightMotorStickyFaults(long bitmask) {}
    public default void clearEncoderStickyFaults(long bitmask) {}
}
