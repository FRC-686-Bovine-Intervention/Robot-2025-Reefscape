package frc.util.loggerUtil.tunables;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;

public class LoggedTunableLinearProfile {
    private final LoggedTunableMeasure<LinearVelocityUnit> maxVelocity;
    private final LoggedTunableMeasure<LinearAccelerationUnit> maxAcceleration;

    public LoggedTunableLinearProfile(String key, Measure<LinearVelocityUnit> maxVelocity, Measure<LinearAccelerationUnit> maxAcceleration) {
        this.maxVelocity = new LoggedTunableMeasure<>(key + "/Max Velocity", maxVelocity);
        this.maxAcceleration = new LoggedTunableMeasure<>(key + "/Max Acceleration", maxAcceleration);
    }

    public boolean hasChanged(int hashCode) {
        return LoggedTunableMeasure.hasChanged(hashCode, maxVelocity, maxAcceleration);
    }

    public void update(MotionMagicConfigs motionMagicConfigs, Measure<DistanceUnit> radius) {
        motionMagicConfigs
            .withMotionMagicCruiseVelocity(RadiansPerSecond.of(maxVelocity.get().div(radius).baseUnitMagnitude()))
            .withMotionMagicAcceleration(RadiansPerSecondPerSecond.of(maxAcceleration.get().div(radius).baseUnitMagnitude()))
        ;
    }

    public TrapezoidProfile getTrapezoidProfile() {
        return new TrapezoidProfile(
            new Constraints(
                maxVelocity.get().in(MetersPerSecond),
                maxAcceleration.get().in(MetersPerSecondPerSecond)
            )
        );
    }

    public void update(ProfiledPIDController profiledPIDController) {
        profiledPIDController.setConstraints(
            new Constraints(
                maxVelocity.get().in(MetersPerSecond),
                maxAcceleration.get().in(MetersPerSecondPerSecond)
            )
        );
    }
}
