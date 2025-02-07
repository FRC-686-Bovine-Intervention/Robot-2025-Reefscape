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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.struct.StructSerializable;
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
        public static final Distance radius = Inches.of(16.5).div(2);
    }


    public static final class Reef {
        public static enum AlgaeLevel {
            High(Meters.of(0.679337), Meters.of(1.313180)),
            Low(Meters.of(0.679337), Meters.of(0.909320)),
            ;
            private final Transform3d transform;
            public final Pose2d forwardRobotSpace;
            public final Pose2d backwardRobotSpace;
            AlgaeLevel(Distance radius, Distance height) {
                this.transform = new Transform3d(
                    new Translation3d(
                        radius.unaryMinus(),
                        Meters.zero(),
                        height
                    ),
                    Rotation3d.kZero
                );
                this.forwardRobotSpace = new Pose2d(
                    new Translation2d(
                        RobotConstants.centerToFrontBumper.plus(minimumReefRadius).minus(radius),
                        height
                    ),
                    Rotation2d.kZero
                );
                this.backwardRobotSpace = new Pose2d(
                    new Translation2d(
                        forwardRobotSpace.getMeasureX().unaryMinus(),
                        forwardRobotSpace.getMeasureY()
                    ),
                    Rotation2d.k180deg.minus(forwardRobotSpace.getRotation())
                );
            }
        }

        public static final Distance minimumReefRadius = Inches.of(65.497).div(2);
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
            Rack0(rackDelta.times(0), AlgaeLevel.High),
            Rack1(rackDelta.times(1), AlgaeLevel.Low),
            Rack2(rackDelta.times(2), AlgaeLevel.High),
            Rack3(rackDelta.times(3), AlgaeLevel.Low),
            Rack4(rackDelta.times(4), AlgaeLevel.High),
            Rack5(rackDelta.times(5), AlgaeLevel.Low),
            ;
            private final Pose2d origin;
            public final AlgaeLevel algaeLevel;
            Rack(Rotation2d rotation, AlgaeLevel algaeLevel) {
                this.origin = new Pose2d(reefCenter.getBlue(), rotation);
                this.algaeLevel = algaeLevel;
            }

            public Flipped<Pose3d> getAlgaePose() {
                return Flipped.fromBlue(new Pose3d(origin).transformBy(algaeLevel.transform));
            }
        }

        public static enum Level {
            Level1(Meters.of(0.592953), Degrees.of(35), Meters.of(0.779254)),
            Level2(Meters.of(0.792953), Degrees.of(35), Meters.of(0.779254)),
            Level3(Meters.of(1.196053), Degrees.of(35), Meters.of(0.779254)),
            Level4(Meters.of(1.828663), Degrees.of(90), Meters.of(0.780750)),
            ;
            private final Transform3d transform;
            public final Pose2d forwardBranchRobotSpace;
            public final Pose2d backwardBranchRobotSpace;
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
                var branchRobotSpace = new Pose2d(
                    new Translation2d(
                        RobotConstants.centerToFrontBumper.plus(minimumReefRadius).minus(radius),
                        height
                    ),
                    new Rotation2d(
                        angle.unaryMinus()
                    )
                );
                this.forwardBranchRobotSpace = branchRobotSpace;
                this.backwardBranchRobotSpace = new Pose2d(
                    new Translation2d(
                        branchRobotSpace.getMeasureX().unaryMinus(),
                        branchRobotSpace.getMeasureY()
                    ),
                    Rotation2d.k180deg.minus(branchRobotSpace.getRotation())
                );
            }
        }

        public static enum Side {
            Left(Meters.of(+0.164308)),
            Right(Meters.of(-0.164309)),
            ;
            private final Transform3d transform;
            private final Transform2d scoringTransform;
            Side(Distance yOffset) {
                this.transform = new Transform3d(
                    new Translation3d(
                        Meters.zero(),
                        yOffset,
                        Meters.zero()
                    ),
                    Rotation3d.kZero
                );
                this.scoringTransform = new Transform2d(
                    new Translation2d(
                        minimumReefRadius.plus(RobotConstants.centerToFrontBumper).unaryMinus(),
                        yOffset
                    ),
                    Rotation2d.kZero
                );
            }
        }

        public static final class Branch implements StructSerializable {
            public final Rack rack;
            public final Level level;
            public final Side side;
            public final Flipped<Pose3d> branchPose;
            public final Flipped<Pose2d> robotPose;

            public Branch(Rack rack, Level level, Side side) {
                this.rack = rack;
                this.level = level;
                this.side = side;
                this.branchPose = Flipped.fromBlue(new Pose3d(rack.origin).transformBy(level.transform).transformBy(side.transform));
                this.robotPose = Flipped.fromBlue(rack.origin.transformBy(side.scoringTransform));
            }

            public int getIndex() {
                return Branch.getIndex(rack, level, side);
            }

            public static int getIndex(Rack rack, Level level, Side side) {
                return (rack.ordinal() * Level.values().length * Side.values().length) + (level.ordinal() * Side.values().length) + (side.ordinal());
            }
        }

        public static final Branch[] branches = new Branch[Rack.values().length * Level.values().length * Side.values().length];
        static {
            for (var rack : Rack.values()) {
                for (var level : Level.values()) {
                    for (var side : Side.values()) {
                        branches[Branch.getIndex(rack, level, side)] = new Branch(rack, level, side);
                    }
                }
            }
        }

        public static final Branch getBranch(Rack rack, Level level, Side side) {
            return branches[Branch.getIndex(rack, level, side)];
        }

        public static final Branch getBranch(int rack, int level, int side) {
            return getBranch(Rack.values()[rack], Level.values()[level], Side.values()[side]);
        }

        public static class StagedAlgae {
            public final Rack rack;
            public final AlgaeLevel algaeLevel;
            public final Flipped<Pose3d> algaePose;
            public final Flipped<Pose2d> robotPose;
            private static final Transform2d scoringTransform = new Transform2d(new Translation2d(minimumReefRadius.plus(RobotConstants.centerToFrontBumper).unaryMinus(), Meters.zero()), Rotation2d.kZero);
            
            public StagedAlgae (Rack rack, AlgaeLevel algaeLevel) {
                this.rack = rack;
                this.algaeLevel = algaeLevel;
                this.algaePose = Flipped.fromBlue(new Pose3d(rack.origin).transformBy(algaeLevel.transform));
                this.robotPose = Flipped.fromBlue(rack.origin.transformBy(scoringTransform));
            }

            public static int getIndex(Rack rack) {
                return rack.ordinal();
            }

            public int getIndex() {
                return StagedAlgae.getIndex(rack);
            }
        }

        public static final StagedAlgae[] stagedAlgae = new StagedAlgae[Rack.values().length];
        static {
            for(var rack : Rack.values()) {
                stagedAlgae[StagedAlgae.getIndex(rack)] = new StagedAlgae(rack, rack.algaeLevel);
            }
        }
    }
}
