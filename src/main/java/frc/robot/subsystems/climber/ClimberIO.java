package frc.robot.subsystems.climber;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.util.NeutralMode;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;
import frc.util.loggerUtil.inputs.LoggedFaults;

public interface ClimberIO {
    @AutoLog
    public class ClimberIOInputs {
        boolean motorConnected = false;
        LoggedEncodedMotor motor = new LoggedEncodedMotor();
        LoggedFaults motorFaults = new LoggedFaults();

        boolean sensor = false;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setVolts(double volts) {}

    public default void setRatchetServoAngle(double angleRads) {}

    public default void setNonClimbingAngle(double angleRads) {}

    public default void setClimbingAngle(double angleRads) {}

    public default void stop(Optional<NeutralMode> neutralMode) {}

    public default void clearMotorStickyFaults(long bitmask) {}
}
