package frc.util.flipping;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import frc.util.geometry.GeomUtil;

public class AllianceFlipUtil {
    public static enum FieldFlipType {
        CenterPointRotation,
        CenterLineMirror,
        XenterLineMirror,
    }
    public static final FieldFlipType defaultFlipType = FieldFlipType.CenterPointRotation;

    public static <T extends AllianceFlippable<T>> T apply(T flippable) {
        return apply(flippable, defaultFlipType);
    }
    public static <T extends AllianceFlippable<T>> T apply(T flippable, FieldFlipType flipType) {
        if (!shouldFlip()) return flippable;
        return flip(flippable, flipType);
    }
    public static <T extends AllianceFlippable<T>> T flip(T flippable) {
        return flip(flippable, defaultFlipType);
    }
    public static <T extends AllianceFlippable<T>> T flip(T flippable, FieldFlipType flipType) {
        return flippable.flip(flipType);
    }
    
    public static Translation2d apply(Translation2d translation) {
        return apply(translation, defaultFlipType);
    }
    public static Translation2d apply(Translation2d translation, FieldFlipType flipType) {
        if (!shouldFlip()) return translation;
        return flip(translation, flipType);
    }
    public static Translation2d flip(Translation2d translation) {
        return flip(translation, defaultFlipType);
    }
    public static Translation2d flip(Translation2d translation, FieldFlipType flipType) {
        switch (flipType) {
            default:
            case CenterPointRotation:   return new Translation2d(FieldConstants.fieldLength.minus(translation.getMeasureX()), FieldConstants.fieldWidth.minus(translation.getMeasureY()));
            case CenterLineMirror:      return new Translation2d(FieldConstants.fieldLength.minus(translation.getMeasureX()), translation.getMeasureY());
            case XenterLineMirror:      return new Translation2d(translation.getMeasureX(), FieldConstants.fieldWidth.minus(translation.getMeasureY()));
        }
    }

    public static Rotation2d apply(Rotation2d rotation) {
        return apply(rotation, defaultFlipType);
    }
    public static Rotation2d apply(Rotation2d rotation, FieldFlipType flipType) {
        if (!shouldFlip()) return rotation;
        return flip(rotation, flipType);
    }
    public static Rotation2d flip(Rotation2d rotation) {
        return flip(rotation, defaultFlipType);
    }
    public static Rotation2d flip(Rotation2d rotation, FieldFlipType flipType) {
        switch (flipType) {
            default:
            case CenterPointRotation:   return new Rotation2d(-rotation.getCos(), -rotation.getSin());
            case CenterLineMirror:      return new Rotation2d(-rotation.getCos(), rotation.getSin());
            case XenterLineMirror:      return new Rotation2d(rotation.getCos(), -rotation.getSin());
        }
    }

    public static Pose2d apply(Pose2d pose) {
        return apply(pose, defaultFlipType);
    }
    public static Pose2d apply(Pose2d pose, FieldFlipType flipType) {
        if (!shouldFlip()) return pose;
        return flip(pose, flipType);
    }
    public static Pose2d flip(Pose2d pose) {
        return flip(pose, defaultFlipType);
    }
    public static Pose2d flip(Pose2d pose, FieldFlipType flipType) {
        return new Pose2d(flip(pose.getTranslation(), flipType), flip(pose.getRotation(), flipType));
    }

    public static Transform2d apply(Transform2d pose) {
        return apply(pose, defaultFlipType);
    }
    public static Transform2d apply(Transform2d pose, FieldFlipType flipType) {
        if (!shouldFlip()) return pose;
        return flip(pose, flipType);
    }
    public static Transform2d flip(Transform2d pose) {
        return flip(pose, defaultFlipType);
    }
    public static Transform2d flip(Transform2d pose, FieldFlipType flipType) {
        return new Transform2d(flip(pose.getTranslation(), flipType), flip(pose.getRotation(), flipType));
    }

    public static Translation3d apply(Translation3d translation) {
        return apply(translation, defaultFlipType);
    }
    public static Translation3d apply(Translation3d translation, FieldFlipType flipType) {
        if (!shouldFlip()) return translation;
        return flip(translation, flipType);
    }
    public static Translation3d flip(Translation3d translation) {
        return flip(translation, defaultFlipType);
    }
    public static Translation3d flip(Translation3d translation, FieldFlipType flipType) {
        switch (flipType) {
            default:
            case CenterPointRotation:   return new Translation3d(FieldConstants.fieldLength.minus(translation.getMeasureX()), FieldConstants.fieldWidth.minus(translation.getMeasureY()), translation.getMeasureZ());
            case CenterLineMirror:      return new Translation3d(FieldConstants.fieldLength.minus(translation.getMeasureX()), translation.getMeasureY(), translation.getMeasureZ());
            case XenterLineMirror:      return new Translation3d(translation.getMeasureX(), FieldConstants.fieldWidth.minus(translation.getMeasureY()), translation.getMeasureZ());
        }
    }
    
    public static Rotation3d apply(Rotation3d rotation) {
        return apply(rotation, defaultFlipType);
    }
    public static Rotation3d apply(Rotation3d rotation, FieldFlipType flipType) {
        if (!shouldFlip()) return rotation;
        return flip(rotation, flipType);
    }
    public static Rotation3d flip(Rotation3d rotation) {
        return flip(rotation, defaultFlipType);
    }
    public static Rotation3d flip(Rotation3d rotation, FieldFlipType flipType) {
        switch (flipType) {
            default:
            case CenterPointRotation:   return rotation.rotateBy(GeomUtil.rotate180Transform3d.getRotation());
            case CenterLineMirror:      return null;
            case XenterLineMirror:      return null;
        }
    }
    
    public static Pose3d apply(Pose3d pose) {
        return apply(pose, defaultFlipType);
    }
    public static Pose3d apply(Pose3d pose, FieldFlipType flipType) {
        if (!shouldFlip()) return pose;
        return flip(pose, flipType);
    }
    public static Pose3d flip(Pose3d pose) {
        return flip(pose, defaultFlipType);
    }
    public static Pose3d flip(Pose3d pose, FieldFlipType flipType) {
        return new Pose3d(flip(pose.getTranslation(), flipType), flip(pose.getRotation(), flipType));
    }
    
    public static Transform3d apply(Transform3d pose) {
        return apply(pose, defaultFlipType);
    }
    public static Transform3d apply(Transform3d pose, FieldFlipType flipType) {
        if (!shouldFlip()) return pose;
        return flip(pose, flipType);
    }
    public static Transform3d flip(Transform3d pose) {
        return flip(pose, defaultFlipType);
    }
    public static Transform3d flip(Transform3d pose, FieldFlipType flipType) {
        return new Transform3d(flip(pose.getTranslation(), flipType), flip(pose.getRotation(), flipType));
    }

    public static ChassisSpeeds applyFieldRelative(ChassisSpeeds speeds) {
        return applyFieldRelative(speeds, defaultFlipType);
    }
    public static ChassisSpeeds applyFieldRelative(ChassisSpeeds speeds, FieldFlipType flipType) {
        if (!shouldFlip()) return speeds;
        return flipFieldRelative(speeds, flipType);
    }
    public static ChassisSpeeds flipFieldRelative(ChassisSpeeds speeds) {
        return flipFieldRelative(speeds, defaultFlipType);
    }
    public static ChassisSpeeds flipFieldRelative(ChassisSpeeds speeds, FieldFlipType flipType) {
        switch (flipType) {
            default:
            case CenterPointRotation:   return new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
            case CenterLineMirror:      return new ChassisSpeeds(-speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
            case XenterLineMirror:      return new ChassisSpeeds(speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        }
    }

    public static ChassisSpeeds applyRobotRelative(ChassisSpeeds speeds, Rotation2d robotRotation) {
        return applyRobotRelative(speeds, robotRotation, defaultFlipType);
    }
    public static ChassisSpeeds applyRobotRelative(ChassisSpeeds speeds, Rotation2d robotRotation, FieldFlipType flipType) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(applyFieldRelative(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, robotRotation)), robotRotation);
    }
    public static ChassisSpeeds flipRobotRelative(ChassisSpeeds speeds, Rotation2d robotRotation, FieldFlipType flipType) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(flipFieldRelative(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, robotRotation), flipType), robotRotation);
    }

    public static boolean shouldFlip() {
        return DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
    }
}
