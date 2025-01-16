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
    private final LoggedTunableMeasure<LinearVelocityUnit> kV;
    private final LoggedTunableMeasure<LinearAccelerationUnit> kA;

    public LoggedTunableLinearProfile(String key,
        Measure<LinearVelocityUnit> kV, Measure<LinearAccelerationUnit> kA
    ) {
        this.kV = new LoggedTunableMeasure<>(key + "/kV", kV);
        this.kA = new LoggedTunableMeasure<>(key + "/kA", kA);
    }

    public boolean hasChanged(int hashCode) {
        return LoggedTunableMeasure.hasChanged(hashCode, kV, kA);
    }

    public void update(MotionMagicConfigs motionMagicConfigs, Measure<DistanceUnit> radius) {
        motionMagicConfigs
            .withMotionMagicCruiseVelocity(RadiansPerSecond.of(kV.get().div(radius).baseUnitMagnitude()))
            .withMotionMagicAcceleration(RadiansPerSecondPerSecond.of(kA.get().div(radius).baseUnitMagnitude()))
        ;
    }

    public TrapezoidProfile getTrapezoidProfile() {
        return new TrapezoidProfile(
            new Constraints(
                kV.get().in(MetersPerSecond),
                kA.get().in(MetersPerSecondPerSecond)
            )
        );
    }

    public void update(ProfiledPIDController profiledPIDController) {
        profiledPIDController.setConstraints(
            new Constraints(
                kV.get().in(MetersPerSecond),
                kA.get().in(MetersPerSecondPerSecond)
            )
        );
    }
}
