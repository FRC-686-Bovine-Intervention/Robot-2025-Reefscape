package frc.util.flipping;

import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.util.flipping.AllianceFlipUtil.FieldFlipType;

public class Flipped<T> {
    private final T blue;
    private final T red;

    private Flipped(T blue, T red) {
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
        if (AllianceFlipUtil.shouldFlip()) {
            return getRed();
        } else {
            return getBlue();
        }
    }
    public T getTheirs() {
        if (!AllianceFlipUtil.shouldFlip()) {
            return getRed();
        } else {
            return getBlue();
        }
    }

    public Flipped<T> map(Function<T, T> mappingFunction) {
        return new Flipped<T>(mappingFunction.apply(this.blue), mappingFunction.apply(this.red));
    }

    public static <T extends Flippable<T>> Flipped<T> fromBlue(T blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static <T extends Flippable<T>> Flipped<T> fromBlue(T blue, FieldFlipType flipType) {
        return new Flipped<T>(blue, AllianceFlipUtil.flip(blue, flipType));
    }
    public static <T extends Flippable<T>> Flipped<T> fromRed(T red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static <T extends Flippable<T>> Flipped<T> fromRed(T red, FieldFlipType flipType) {
        return new Flipped<T>(AllianceFlipUtil.flip(red, flipType), red);
    }

    public static Flipped<Translation2d> fromBlue(Translation2d blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static Flipped<Translation2d> fromBlue(Translation2d blue, FieldFlipType flipType) {
        return new Flipped<Translation2d>(blue, AllianceFlipUtil.flip(blue, flipType));
    }
    public static Flipped<Translation2d> fromRed(Translation2d red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static Flipped<Translation2d> fromRed(Translation2d red, FieldFlipType flipType) {
        return new Flipped<Translation2d>(AllianceFlipUtil.flip(red, flipType), red);
    }

    public static Flipped<Rotation2d> fromBlue(Rotation2d blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static Flipped<Rotation2d> fromBlue(Rotation2d blue, FieldFlipType flipType) {
        return new Flipped<Rotation2d>(blue, AllianceFlipUtil.flip(blue, flipType));
    }
    public static Flipped<Rotation2d> fromRed(Rotation2d red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static Flipped<Rotation2d> fromRed(Rotation2d red, FieldFlipType flipType) {
        return new Flipped<Rotation2d>(AllianceFlipUtil.flip(red, flipType), red);
    }

    public static Flipped<Pose2d> fromBlue(Pose2d blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static Flipped<Pose2d> fromBlue(Pose2d blue, FieldFlipType flipType) {
        return new Flipped<Pose2d>(blue, AllianceFlipUtil.flip(blue, flipType));
    }
    public static Flipped<Pose2d> fromRed(Pose2d red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static Flipped<Pose2d> fromRed(Pose2d red, FieldFlipType flipType) {
        return new Flipped<Pose2d>(AllianceFlipUtil.flip(red, flipType), red);
    }

    public static Flipped<Transform2d> fromBlue(Transform2d blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static Flipped<Transform2d> fromBlue(Transform2d blue, FieldFlipType flipType) {
        return new Flipped<Transform2d>(blue, AllianceFlipUtil.flip(blue, flipType));
    }
    public static Flipped<Transform2d> fromRed(Transform2d red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static Flipped<Transform2d> fromRed(Transform2d red, FieldFlipType flipType) {
        return new Flipped<Transform2d>(AllianceFlipUtil.flip(red, flipType), red);
    }

    public static Flipped<Translation3d> fromBlue(Translation3d blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static Flipped<Translation3d> fromBlue(Translation3d blue, FieldFlipType flipType) {
        return new Flipped<Translation3d>(blue, AllianceFlipUtil.flip(blue, flipType));
    }
    public static Flipped<Translation3d> fromRed(Translation3d red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static Flipped<Translation3d> fromRed(Translation3d red, FieldFlipType flipType) {
        return new Flipped<Translation3d>(AllianceFlipUtil.flip(red, flipType), red);
    }

    public static Flipped<Rotation3d> fromBlue(Rotation3d blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static Flipped<Rotation3d> fromBlue(Rotation3d blue, FieldFlipType flipType) {
        return new Flipped<Rotation3d>(blue, AllianceFlipUtil.flip(blue, flipType));
    }
    public static Flipped<Rotation3d> fromRed(Rotation3d red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static Flipped<Rotation3d> fromRed(Rotation3d red, FieldFlipType flipType) {
        return new Flipped<Rotation3d>(AllianceFlipUtil.flip(red, flipType), red);
    }

    public static Flipped<Pose3d> fromBlue(Pose3d blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static Flipped<Pose3d> fromBlue(Pose3d blue, FieldFlipType flipType) {
        return new Flipped<Pose3d>(blue, AllianceFlipUtil.flip(blue, flipType));
    }
    public static Flipped<Pose3d> fromRed(Pose3d red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static Flipped<Pose3d> fromRed(Pose3d red, FieldFlipType flipType) {
        return new Flipped<Pose3d>(AllianceFlipUtil.flip(red, flipType), red);
    }

    public static Flipped<Transform3d> fromBlue(Transform3d blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static Flipped<Transform3d> fromBlue(Transform3d blue, FieldFlipType flipType) {
        return new Flipped<Transform3d>(blue, AllianceFlipUtil.flip(blue, flipType));
    }
    public static Flipped<Transform3d> fromRed(Transform3d red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static Flipped<Transform3d> fromRed(Transform3d red, FieldFlipType flipType) {
        return new Flipped<Transform3d>(AllianceFlipUtil.flip(red, flipType), red);
    }
}
