package frc.robot.subsystems.drive;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;
import frc.util.loggerUtil.inputs.LoggedEncoder;
import frc.util.loggerUtil.inputs.LoggedFaults;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        boolean driveMotorConnected = false;
        boolean azimuthMotorConnected = false;
        boolean azimuthEncoderConnected = false;
        LoggedEncodedMotor driveMotor = new LoggedEncodedMotor();
        LoggedEncodedMotor azimuthMotor = new LoggedEncodedMotor();
        LoggedEncoder azimuthEncoder = new LoggedEncoder();
        LoggedFaults driveMotorFaults = new LoggedFaults();
        LoggedFaults azimuthMotorFaults = new LoggedFaults();

        double[] odometryDriveRads = new double[0];
        double[] odometryAzimuthRads = new double[0];
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /** Run the drive motor at the specified voltage. */
    public default void setDriveVoltage(Measure<VoltageUnit> voltage) {}
    public default void setDriveVelocity(Measure<AngularVelocityUnit> velocity, Measure<AngularAccelerationUnit> acceleration, Measure<VoltageUnit> feedforward, boolean overrideWithBrakeMode) {}

    /** Run the turn motor at the specified voltage. */
    public default void setAzimuthVoltage(Measure<VoltageUnit> volts) {}
    public default void setAzimuthAngle(Measure<AngleUnit> angle) {}

    public default void stopDrive(Optional<NeutralMode> neutralMode) {}
    public default void stopAzimuth(Optional<NeutralMode> neutralMode) {}

    public default void configDrivePID(PIDConstants pidConstants) {}
    public default void configAzimuthPID(PIDConstants pidConstants) {}

    public default void clearDriveStickyFaults(long bitmask) {}
    public default void clearAzimuthStickyFaults(long bitmask) {}
}
