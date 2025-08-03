package frc.robot.subsystems.drive.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;

public class AutoScore {
    private static final Supplier<Transform2d> pretargetTransform = new Supplier<>() {
        private static final LoggedTunableMeasure<DistanceUnit> pretargetDistance = new LoggedTunableMeasure<>("Auto Score/Pretarget Distance", Inches.of(12));
        
        private Transform2d cache;
        public Transform2d get() {
            if (LoggedTunableMeasure.hasChanged(this.hashCode(), pretargetDistance)) {
                this.cache = new Transform2d(new Translation2d(pretargetDistance.get().unaryMinus().in(Meters), 0), Rotation2d.kZero);
            }
            return this.cache;
        }
    };
    private static final LoggedTunableNumber longDistanceAlphaScalar = new LoggedTunableNumber("Auto Score/Long Distance Alpha Scalar", 0.3);
    private static final LoggedTunableNumber finalKp = new LoggedTunableNumber("Auto Score/Final kP", 5);
    private static final LoggedTunableNumber angularKp = new LoggedTunableNumber("Auto Score/Angular kP", 3);
    private static final LoggedTunableNumber shortDistanceAlphaScalar = new LoggedTunableNumber("Auto Score/Short Distance Alpha Scalar", 1);
    private static final LoggedTunableMeasure<LinearAccelerationUnit> maxLinearAcceleration = new LoggedTunableMeasure<>("Auto Score/Max Linear Acceleration", MetersPerSecondPerSecond.of(5));
    // private static final LoggedTunableMeasure<AngularAccelerationUnit> maxAngularAcceleration = new LoggedTunableMeasure<>("Auto Score/Max Angular Acceleration", RotationsPerSecondPerSecond.of(1));
    private static final LoggedTunableMeasure<AngleUnit> angularTolerance = new LoggedTunableMeasure<>("Auto Score/Angular Threshold", Degrees.of(5));
    private static final LoggedTunableMeasure<DistanceUnit> linearThreshold = new LoggedTunableMeasure<>("Auto Score/Linear Threshold", Inches.of(24));

    public static Command pilotDriveToReef(Drive drive, Supplier<Pose2d> finalTargetPoseSupplier) {
        return new Command() {
            {
                addRequirements(drive.subsystems);
                setName("Pilot Drive");
            }
            @Override
            public void initialize() {
                // TODO Auto-generated method stub
                super.initialize();
            }
            @Override
            public void execute() {
                var currentPose = drive.getPose();
                var finalTargetPose = finalTargetPoseSupplier.get();
                var pretargetPose = finalTargetPose.transformBy(pretargetTransform.get());
                var currentToPretarget = pretargetPose.getTranslation().minus(currentPose.getTranslation());
                var pretargetToTarget = finalTargetPose.getTranslation().minus(pretargetPose.getTranslation());
                
                var currentToFinalTarget = finalTargetPose.getTranslation().minus(currentPose.getTranslation());
                var velocityToFinalMagnitude = currentToFinalTarget.getNorm() * finalKp.get();

                var angularError = pretargetPose.getRotation().minus(currentPose.getRotation());
                var angleWithinTolerance = angularError.getCos() >= Math.cos(angularTolerance.get().in(Radians));

                double velocityAtPretargetMagnitude;
                if (angleWithinTolerance) {
                    velocityAtPretargetMagnitude = velocityToFinalMagnitude;
                } else {
                    velocityAtPretargetMagnitude = 0;
                }
                var velocityToPretargetMagnitude = Math.sqrt(
                    Math.abs(
                        (velocityAtPretargetMagnitude * velocityAtPretargetMagnitude)
                        - (
                            2 * maxLinearAcceleration.get().in(MetersPerSecondPerSecond)
                            * currentToPretarget.getNorm()
                        )
                    )
                );

                Rotation2d velocityDir;
                double velocityMagnitude;
                if (currentToPretarget.getNorm() + currentToFinalTarget.getNorm() <= linearThreshold.get().in(Meters)) {
                    if (angleWithinTolerance) {
                        velocityDir = currentToFinalTarget.getAngle();
                        velocityMagnitude = velocityToFinalMagnitude;
                    } else {
                        velocityDir = currentToPretarget.getAngle();
                        velocityMagnitude = velocityToPretargetMagnitude;
                    }
                } else {
                    var alpha = currentToPretarget.getAngle().minus(pretargetToTarget.getAngle());
                    velocityDir = currentToPretarget.getAngle().plus(alpha.times(shortDistanceAlphaScalar.get()));
                    velocityMagnitude = velocityToPretargetMagnitude;
                }
                
                var angularVelocity = angularError.getRadians() * angularKp.get();

                var fieldVelocity = new ChassisSpeeds(
                    velocityMagnitude * velocityDir.getCos(),
                    velocityMagnitude * velocityDir.getSin(),
                    angularVelocity
                );

                drive.runFieldSpeeds(fieldVelocity);
            }
            @Override
            public void end(boolean interrupted) {
                // TODO Auto-generated method stub
                super.end(interrupted);
            }
        };
    }
}
