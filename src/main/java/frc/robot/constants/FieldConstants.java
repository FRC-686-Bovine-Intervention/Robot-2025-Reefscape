package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.util.flipping.Flipped;

public final class FieldConstants {
    public static final Distance fieldLength = Inches.of(57*12 + 6 + 7.0/8.0);
    public static final Distance fieldWidth =  Inches.of(26*12 + 5);

    public static final AprilTagFieldLayout apriltagLayout;
    static {
        AprilTagFieldLayout a = null;
        try {
            a = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch(Exception e) {
            e.printStackTrace();
        }
        apriltagLayout = a;
    }

    public static final class Coral {
        public static final Distance length = Inches.of(11.875);
        public static final Distance radius = Inches.of(2);

        public static final Transform3d rackPlacement = new Transform3d(
            new Translation3d(
                length.div(2).minus(Inches.of(2)),
                Inches.zero(),
                Inches.zero()
            ),
            Rotation3d.kZero
        );
        public static final Transform3d standUp = new Transform3d(
            new Translation3d(
                Inches.zero(),
                Inches.zero(),
                length.div(2)
            ),
            new Rotation3d(
                Degrees.zero(),
                Degrees.of(-90),
                Degrees.zero()
            )
        );
    }

    public static final class Algae {
        public static final Distance radius = Inches.of(0);
    }


    public static final class Reef {
        public static final Flipped<Translation2d> reefCenter = Flipped.fromBlue(
            new Translation2d(
                Meters.of(4.489325),
                Meters.of(4.025877)
            )
        );
        private static final Rotation2d rackDelta = new Rotation2d(
            Degrees.of(60)
        );
        public static enum Rack {
            Rack0(rackDelta.times(0)),
            Rack1(rackDelta.times(1)),
            Rack2(rackDelta.times(2)),
            Rack3(rackDelta.times(3)),
            Rack4(rackDelta.times(4)),
            Rack5(rackDelta.times(5)),
            ;
            private final Pose2d origin;
            Rack(Rotation2d rotation) {
                this.origin = new Pose2d(reefCenter.getBlue(), rotation);
            }
        }

        public static enum Level {
            Level1(Meters.of(0.592953), Degrees.of(35), Meters.of(0.779254)),
            Level2(Meters.of(0.792953), Degrees.of(35), Meters.of(0.779254)),
            Level3(Meters.of(1.196053), Degrees.of(35), Meters.of(0.779254)),
            Level4(Meters.of(1.828663), Degrees.of(90), Meters.of(0.780750)),
            ;
            private final Transform3d transform;
            Level(Distance height, Angle angle, Distance radius) {
                this.transform = new Transform3d(
                    new Translation3d(
                        radius.unaryMinus(),
                        Meters.zero(),
                        height
                    ),
                    new Rotation3d(
                        Degrees.zero(),
                        angle,
                        Degrees.zero()
                    )
                );
            }
        }

        public static enum Side {
            Left(Meters.of(+0.164308)),
            Right(Meters.of(-0.164309)),
            ;
            private final Transform3d transform;
            Side(Distance yOffset) {
                this.transform = new Transform3d(
                    new Translation3d(
                        Meters.zero(),
                        yOffset,
                        Meters.zero()
                    ),
                    Rotation3d.kZero
                );
            }
        }

        public static final class Node {
            public final Flipped<Pose3d> pose;
            public final Rack rack;
            public final Level level;
            public final Side side;

            Node(Rack rack, Level level, Side side) {
                this.rack = rack;
                this.level = level;
                this.side = side;
                this.pose = Flipped.fromBlue(new Pose3d(rack.origin).transformBy(level.transform).transformBy(side.transform));
            }
        }

        public static final Node[] nodes = new Node[Rack.values().length * Level.values().length * Side.values().length];
        static {
            for (var rack : Rack.values()) {
                for (var level : Level.values()) {
                    for (var side : Side.values()) {
                        nodes[(rack.ordinal() * Level.values().length * Side.values().length) + (level.ordinal() * Side.values().length) + (side.ordinal())] = 
                            new Node(rack, level, side)
                        ;
                    }
                }
            }
        }

        public static final Node getNode(Rack rack, Level level, Side side) {
            return nodes[(rack.ordinal() * Level.values().length * Side.values().length) + (level.ordinal() * Side.values().length) + (side.ordinal())];
        }
    }
}
