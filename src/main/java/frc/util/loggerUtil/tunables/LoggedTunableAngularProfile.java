package frc.util.loggerUtil.tunables;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;

public class LoggedTunableAngularProfile {
    private final LoggedTunableMeasure<AngularVelocityUnit> maxVelocity;
    private final LoggedTunableMeasure<AngularAccelerationUnit> maxAcceleration;

    public LoggedTunableAngularProfile(String key, Measure<AngularVelocityUnit> maxVelocity, Measure<AngularAccelerationUnit> maxAcceleration) {
        this.maxVelocity = new LoggedTunableMeasure<>(key + "/Max Velocity", maxVelocity);
        this.maxAcceleration = new LoggedTunableMeasure<>(key + "/Max Acceleration", maxAcceleration);
    }

    public boolean hasChanged(int hashCode) {
        return LoggedTunableMeasure.hasChanged(hashCode, maxVelocity, maxAcceleration);
    }

    public void update(MotionMagicConfigs motionMagicConfigs) {
        motionMagicConfigs
            .withMotionMagicCruiseVelocity(maxVelocity.in(RotationsPerSecond))
            .withMotionMagicAcceleration(maxAcceleration.in(RotationsPerSecondPerSecond))
        ;
    }

    public TrapezoidProfile getTrapezoidProfile() {
        return new TrapezoidProfile(
            new Constraints(
                maxVelocity.get().in(RadiansPerSecond),
                maxAcceleration.get().in(RadiansPerSecondPerSecond)
            )
        );
    }

    public void update(ProfiledPIDController profiledPIDController) {
        profiledPIDController.setConstraints(
            new Constraints(
                maxVelocity.get().in(RadiansPerSecond),
                maxAcceleration.get().in(RadiansPerSecondPerSecond)
            )
        );
    }
}
