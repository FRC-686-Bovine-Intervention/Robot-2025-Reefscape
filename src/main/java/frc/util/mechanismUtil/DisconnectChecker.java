package frc.util.mechanismUtil;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.util.loggerUtil.inputs.LoggedEncoder;

public class DisconnectChecker {
    private final GearRatio gearRatio;
    private final Rotation2d positionTolerance;
    private final Rotation2d rotorOffset;
    private final AngularVelocity velocityTolerance;
    private final Timer positionDisconnectTimer = new Timer();
    private final Timer velocityDisconnectTimer = new Timer();

    public DisconnectChecker(GearRatio gearRatio, Angle rotorOffset, Angle positionTolerance, AngularVelocity velocityTolerance) {
        this.gearRatio = gearRatio;
        this.rotorOffset = new Rotation2d(rotorOffset);
        this.positionTolerance = new Rotation2d(positionTolerance);
        this.velocityTolerance = velocityTolerance;
    }

    public void update(LoggedEncoder encoder1, LoggedEncoder encoder2) {
        update(encoder1.position, encoder1.velocity, encoder2.position, encoder2.velocity);
    }
    public void update(Angle position1, AngularVelocity velocity1, Angle position2, AngularVelocity velocity2) {
        var positionError = new Rotation2d(gearRatio.applySigned(position1)).minus(new Rotation2d(position2)).minus(rotorOffset);
        var withinPositionTolerance = positionError.getCos() > positionTolerance.getCos();
        var velocityError = velocity1.minus(velocity2);
        var withinVelocityTolerance = velocityError.lte(velocityTolerance);

        Logger.recordOutput("DisconnectChecker/Position Error", positionError.getMeasure());
        Logger.recordOutput("DisconnectChecker/Velocity Error", velocityError);
        Logger.recordOutput("DisconnectChecker/Within Position Tolerance", withinPositionTolerance);
        Logger.recordOutput("DisconnectChecker/Within Velocity Tolerance", withinVelocityTolerance);
    }
}
