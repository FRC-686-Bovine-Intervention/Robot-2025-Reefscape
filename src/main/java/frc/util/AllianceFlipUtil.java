package frc.util;

import static edu.wpi.first.units.Units.Meters;

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

public class AllianceFlipUtil {
    public static enum FieldFlipType {
        CenterPointFlip,
        MirrorFlip,
    }
    public static final FieldFlipType defaultFlipType = FieldFlipType.CenterPointFlip;

    public static Translation2d apply(Translation2d translation) {
        return apply(translation, defaultFlipType);
    }
    public static Translation2d apply(Translation2d translation, FieldFlipType flipType) {
        if(!shouldFlip()) return translation;
        return flip(translation, flipType);
    }
    public static Translation2d flip(Translation2d translation) {
        return flip(translation, defaultFlipType);
    }
    public static Translation2d flip(Translation2d translation, FieldFlipType flipType) {
        switch(flipType) {
            default:
            case CenterPointFlip: return new Translation2d(FieldConstants.fieldLength.in(Meters) - translation.getX(), FieldConstants.fieldWidth.in(Meters) - translation.getY());
            case MirrorFlip:      return new Translation2d(FieldConstants.fieldLength.in(Meters) - translation.getX(), translation.getY());
        }
    }

    public static Rotation2d apply(Rotation2d rotation) {
        return apply(rotation, defaultFlipType);
    }
    public static Rotation2d apply(Rotation2d rotation, FieldFlipType flipType) {
        if(!shouldFlip()) return rotation;
        return flip(rotation, flipType);
    }
    public static Rotation2d flip(Rotation2d rotation) {
        return flip(rotation, defaultFlipType);
    }
    public static Rotation2d flip(Rotation2d rotation, FieldFlipType flipType) {
        switch(flipType) {
            default:
            case CenterPointFlip: return rotation.rotateBy(Rotation2d.fromRotations(0.5));
            case MirrorFlip:      return new Rotation2d(-rotation.getCos(), rotation.getSin());
        }
    }

    public static Pose2d apply(Pose2d pose) {
        return apply(pose, defaultFlipType);
    }
    public static Pose2d apply(Pose2d pose, FieldFlipType flipType) {
        if(!shouldFlip()) return pose;
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
        if(!shouldFlip()) return pose;
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
        if(!shouldFlip()) return translation;
        return flip(translation, flipType);
    }
    public static Translation3d flip(Translation3d translation) {
        return flip(translation, defaultFlipType);
    }
    public static Translation3d flip(Translation3d translation, FieldFlipType flipType) {
        switch(flipType) {
            default:
            case CenterPointFlip: return new Translation3d(FieldConstants.fieldLength.in(Meters) - translation.getX(), FieldConstants.fieldWidth.in(Meters) - translation.getY(), translation.getZ());
            case MirrorFlip:      return new Translation3d(FieldConstants.fieldLength.in(Meters) - translation.getX(), translation.getY(), translation.getZ());
        }
    }
    
    public static Rotation3d apply(Rotation3d rotation) {
        return apply(rotation, defaultFlipType);
    }
    public static Rotation3d apply(Rotation3d rotation, FieldFlipType flipType) {
        if(!shouldFlip()) return rotation;
        return flip(rotation, flipType);
    }
    public static Rotation3d flip(Rotation3d rotation) {
        return flip(rotation, defaultFlipType);
    }
    private static final Rotation3d rev = new Rotation3d(Rotation2d.k180deg);
    public static Rotation3d flip(Rotation3d rotation, FieldFlipType flipType) {
        switch(flipType) {
            default:
            case CenterPointFlip: return rotation.rotateBy(rev);
            case MirrorFlip:      return null;
        }
    }
    
    public static Pose3d apply(Pose3d pose) {
        return apply(pose, defaultFlipType);
    }
    public static Pose3d apply(Pose3d pose, FieldFlipType flipType) {
        if(!shouldFlip()) return pose;
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
        if(!shouldFlip()) return pose;
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
        if(!shouldFlip()) return speeds;
        return flipFieldRelative(speeds, flipType);
    }
    public static ChassisSpeeds flipFieldRelative(ChassisSpeeds speeds) {
        return flipFieldRelative(speeds, defaultFlipType);
    }
    public static ChassisSpeeds flipFieldRelative(ChassisSpeeds speeds, FieldFlipType flipType) {
        switch (flipType) {
            default:
            case CenterPointFlip: return new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
            case MirrorFlip: return new ChassisSpeeds(-speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
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

    public static class FlippedGeometry<T> {
        private final T blue;
        private final T red;

        private FlippedGeometry(T blue, T red) {
            this.blue = blue;
            this.red = red;
        }

        public T getBlue() {
            return blue;
        }
        public T getRed() {
            return red;
        }

        public T getOurs() {
            if(shouldFlip()) {
                return getRed();
            } else {
                return getBlue();
            }
        }
        public T getTheirs() {
            if(!shouldFlip()) {
                return getRed();
            } else {
                return getBlue();
            }
        }

        public static FlippedGeometry<Translation2d> fromBlue(Translation2d blue) {
            return fromBlue(blue, defaultFlipType);
        }
        public static FlippedGeometry<Translation2d> fromBlue(Translation2d blue, FieldFlipType flipType) {
            return new FlippedGeometry<Translation2d>(blue, flip(blue, flipType));
        }
        public static FlippedGeometry<Translation2d> fromRed(Translation2d red) {
            return fromRed(red, defaultFlipType);
        }
        public static FlippedGeometry<Translation2d> fromRed(Translation2d red, FieldFlipType flipType) {
            return new FlippedGeometry<Translation2d>(flip(red, flipType), red);
        }

        public static FlippedGeometry<Rotation2d> fromBlue(Rotation2d blue) {
            return fromBlue(blue, defaultFlipType);
        }
        public static FlippedGeometry<Rotation2d> fromBlue(Rotation2d blue, FieldFlipType flipType) {
            return new FlippedGeometry<Rotation2d>(blue, flip(blue, flipType));
        }
        public static FlippedGeometry<Rotation2d> fromRed(Rotation2d red) {
            return fromRed(red, defaultFlipType);
        }
        public static FlippedGeometry<Rotation2d> fromRed(Rotation2d red, FieldFlipType flipType) {
            return new FlippedGeometry<Rotation2d>(flip(red, flipType), red);
        }

        public static FlippedGeometry<Pose2d> fromBlue(Pose2d blue) {
            return fromBlue(blue, defaultFlipType);
        }
        public static FlippedGeometry<Pose2d> fromBlue(Pose2d blue, FieldFlipType flipType) {
            return new FlippedGeometry<Pose2d>(blue, flip(blue, flipType));
        }
        public static FlippedGeometry<Pose2d> fromRed(Pose2d red) {
            return fromRed(red, defaultFlipType);
        }
        public static FlippedGeometry<Pose2d> fromRed(Pose2d red, FieldFlipType flipType) {
            return new FlippedGeometry<Pose2d>(flip(red, flipType), red);
        }

        public static FlippedGeometry<Transform2d> fromBlue(Transform2d blue) {
            return fromBlue(blue, defaultFlipType);
        }
        public static FlippedGeometry<Transform2d> fromBlue(Transform2d blue, FieldFlipType flipType) {
            return new FlippedGeometry<Transform2d>(blue, flip(blue, flipType));
        }
        public static FlippedGeometry<Transform2d> fromRed(Transform2d red) {
            return fromRed(red, defaultFlipType);
        }
        public static FlippedGeometry<Transform2d> fromRed(Transform2d red, FieldFlipType flipType) {
            return new FlippedGeometry<Transform2d>(flip(red, flipType), red);
        }

        public static FlippedGeometry<Translation3d> fromBlue(Translation3d blue) {
            return fromBlue(blue, defaultFlipType);
        }
        public static FlippedGeometry<Translation3d> fromBlue(Translation3d blue, FieldFlipType flipType) {
            return new FlippedGeometry<Translation3d>(blue, flip(blue, flipType));
        }
        public static FlippedGeometry<Translation3d> fromRed(Translation3d red) {
            return fromRed(red, defaultFlipType);
        }
        public static FlippedGeometry<Translation3d> fromRed(Translation3d red, FieldFlipType flipType) {
            return new FlippedGeometry<Translation3d>(flip(red, flipType), red);
        }

        public static FlippedGeometry<Rotation3d> fromBlue(Rotation3d blue) {
            return fromBlue(blue, defaultFlipType);
        }
        public static FlippedGeometry<Rotation3d> fromBlue(Rotation3d blue, FieldFlipType flipType) {
            return new FlippedGeometry<Rotation3d>(blue, flip(blue, flipType));
        }
        public static FlippedGeometry<Rotation3d> fromRed(Rotation3d red) {
            return fromRed(red, defaultFlipType);
        }
        public static FlippedGeometry<Rotation3d> fromRed(Rotation3d red, FieldFlipType flipType) {
            return new FlippedGeometry<Rotation3d>(flip(red, flipType), red);
        }

        public static FlippedGeometry<Pose3d> fromBlue(Pose3d blue) {
            return fromBlue(blue, defaultFlipType);
        }
        public static FlippedGeometry<Pose3d> fromBlue(Pose3d blue, FieldFlipType flipType) {
            return new FlippedGeometry<Pose3d>(blue, flip(blue, flipType));
        }
        public static FlippedGeometry<Pose3d> fromRed(Pose3d red) {
            return fromRed(red, defaultFlipType);
        }
        public static FlippedGeometry<Pose3d> fromRed(Pose3d red, FieldFlipType flipType) {
            return new FlippedGeometry<Pose3d>(flip(red, flipType), red);
        }

        public static FlippedGeometry<Transform3d> fromBlue(Transform3d blue) {
            return fromBlue(blue, defaultFlipType);
        }
        public static FlippedGeometry<Transform3d> fromBlue(Transform3d blue, FieldFlipType flipType) {
            return new FlippedGeometry<Transform3d>(blue, flip(blue, flipType));
        }
        public static FlippedGeometry<Transform3d> fromRed(Transform3d red) {
            return fromRed(red, defaultFlipType);
        }
        public static FlippedGeometry<Transform3d> fromRed(Transform3d red, FieldFlipType flipType) {
            return new FlippedGeometry<Transform3d>(flip(red, flipType), red);
        }
    }
}
