package frc.robot.subsystems.drive.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;

public class AutoScore {
    private static final Supplier<Transform2d> pretargetTransform = new Supplier<>() {
        private static final LoggedTunableMeasure<DistanceUnit> pretargetDistance = new LoggedTunableMeasure<>("Auto Score/Pretarget Distance", Inches.of(24));
        
        private Transform2d cache;
        public Transform2d get() {
            if (LoggedTunableMeasure.hasChanged(this.hashCode(), pretargetDistance)) {
                this.cache = new Transform2d(new Translation2d(pretargetDistance.get().unaryMinus().in(Meters), 0), Rotation2d.kZero);
            }
            return this.cache;
        }
    };
    private static final LoggedTunableNumber alphaScalar = new LoggedTunableNumber("Auto Score/Alpha Scalar", 1);
    private static final LoggedTunableMeasure<LinearAccelerationUnit> maxAcceleration = new LoggedTunableMeasure<>("Auto Score/Max Acceleration", MetersPerSecondPerSecond.of(5));

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
                var pretargetToTarget = pretargetPose.getTranslation().minus(finalTargetPose.getTranslation());
                var alpha = currentToPretarget.getAngle().minus(pretargetToTarget.getAngle());
                var velocityDir = currentToPretarget.getAngle().plus(alpha.times(alphaScalar.get()));
                var velocityMagnitude = Math.sqrt(2 * maxAcceleration.get().in(MetersPerSecondPerSecond) * currentToPretarget.getNorm());
                var fieldVelocity = new ChassisSpeeds(
                    velocityMagnitude * velocityDir.getCos(),
                    velocityMagnitude * velocityDir.getSin(),
                    0
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
