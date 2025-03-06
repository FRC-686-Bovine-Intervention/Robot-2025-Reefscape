package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

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
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.superstructure.Superstructure.RobotFlippedRobotPose;
import frc.robot.subsystems.superstructure.Superstructure.RobotFlippedSuperstructureState;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.pivot.PivotConstants;
import frc.util.flipping.AllianceFlipUtil;
import frc.util.flipping.AllianceFlipUtil.FieldFlipType;
import frc.util.flipping.AllianceFlipped;

public final class FieldConstants {
    public static final Distance fieldLength = Inches.of(57*12 + 6 + 7.0/8.0);
    public static final Distance fieldWidth =  Inches.of(26*12 + 5);

    public static final AprilTagFieldLayout apriltagLayout;
    static {
        AprilTagFieldLayout a = null;
        try {
            a = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        } catch(Exception e) {
            e.printStackTrace();
        }
        apriltagLayout = a;
    }

    public static final AllianceFlipped<Rotation2d> netForwardRotation = AllianceFlipped.fromBlue(Rotation2d.kZero);

    public static final class Coral {
        public static final Distance length = Inches.of(11.875);
        public static final Distance radius = Inches.of(4.5).div(2);

        public static final Transform3d branchPlacement = new Transform3d(
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

    public static final class CoralStation {
        private static final Angle rightStationWallAngle = Degrees.of(144.011392);
        private static final Pose2d rightStationMidpoint = new Pose2d(
            new Translation2d(
                Inches.of(33.563385),
                Inches.of(25.876826)
            ),
            new Rotation2d(rightStationWallAngle).minus(Rotation2d.kCCW_90deg)
        );
        private static final Pose2d leftStationMidpoint = AllianceFlipUtil.flip(rightStationMidpoint, FieldFlipType.XenterLineMirror);
        private static final Transform2d centerStationTransform = new Transform2d(
            new Translation2d(
                RobotConstants.centerToFrontBumper.minus(Inches.of(6)),
                Inches.zero()
            ),
            Rotation2d.kZero
        );
        private static final Transform2d leftStationTransform = new Transform2d(
            new Translation2d(
                RobotConstants.centerToFrontBumper.minus(Inches.of(6)),
                Inches.of(24)
            ),
            Rotation2d.kZero
        );
        private static final Transform2d rightStationTransform = new Transform2d(
            new Translation2d(
                RobotConstants.centerToFrontBumper.minus(Inches.of(6)),
                Inches.of(24).unaryMinus()
            ),
            Rotation2d.kZero
        );

        public static final AllianceFlipped<RobotFlippedRobotPose> leftStationLeft = AllianceFlipped.fromBlue(RobotFlippedRobotPose.fromBackwardRobotFlipped(leftStationMidpoint.transformBy(leftStationTransform)));
        public static final AllianceFlipped<RobotFlippedRobotPose> leftStationCenter = AllianceFlipped.fromBlue(RobotFlippedRobotPose.fromBackwardRobotFlipped(leftStationMidpoint.transformBy(centerStationTransform)));
        public static final AllianceFlipped<RobotFlippedRobotPose> leftStationRight = AllianceFlipped.fromBlue(RobotFlippedRobotPose.fromBackwardRobotFlipped(leftStationMidpoint.transformBy(rightStationTransform)));

        public static final AllianceFlipped<RobotFlippedRobotPose> rightStationLeft = AllianceFlipped.fromBlue(RobotFlippedRobotPose.fromBackwardRobotFlipped(rightStationMidpoint.transformBy(leftStationTransform)));
        public static final AllianceFlipped<RobotFlippedRobotPose> rightStationCenter = AllianceFlipped.fromBlue(RobotFlippedRobotPose.fromBackwardRobotFlipped(rightStationMidpoint.transformBy(centerStationTransform)));
        public static final AllianceFlipped<RobotFlippedRobotPose> rightStationRight = AllianceFlipped.fromBlue(RobotFlippedRobotPose.fromBackwardRobotFlipped(rightStationMidpoint.transformBy(rightStationTransform)));

        public static final Angle chuteAngle = Degrees.of(35);
        public static final Distance chuteBottomHeight = Inches.of(37.440179);

        public static final RobotFlippedSuperstructureState intakePosition = new RobotFlippedSuperstructureState(
            SuperstructureState.fromRobotSpace(
                new Pose2d(
                    new Translation2d(
                        RobotConstants.centerToFrontBumper,
                        chuteBottomHeight.plus(Coral.radius.times(Math.cos(chuteAngle.in(Radians))))
                    ),
                    new Rotation2d(chuteAngle)
                )
                .transformBy(SuperstructureConstants.coralIntakeForwardTransform)
            ),
            SuperstructureState.fromParts(
                Degrees.of(85),
                ElevatorConstants.minLengthPhysical,
                Degrees.of(160)
            )
        );
    }

    public static final class Reef {
        public static final Distance minimumReefRadius = Inches.of(65.497).div(2);
        public static final AllianceFlipped<Translation2d> reefCenter = AllianceFlipped.fromBlue(
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
            private final Transform2d scoringTransform = new Transform2d(new Translation2d(minimumReefRadius.plus(RobotConstants.centerToFrontBumper).unaryMinus(), Meters.zero()), Rotation2d.kZero);
            private final Pose2d origin;
            
            public final Pipe leftPipe;
            public final Pipe rightPipe;
            public final StagedAlgae stagedAlgae;

            public final AllianceFlipped<RobotFlippedRobotPose> algaeIntakeRobotPose;

            Rack(Rotation2d rotation, AlgaeLevel algaeLevel) {
                this.origin = new Pose2d(reefCenter.getBlue(), rotation);
                this.leftPipe = new Pipe(this, Side.Left);
                this.rightPipe = new Pipe(this, Side.Right);
                this.stagedAlgae = new StagedAlgae(this, algaeLevel);
                this.algaeIntakeRobotPose = AllianceFlipped.fromBlue(RobotFlippedRobotPose.fromForwardRobotFlipped(origin.transformBy(scoringTransform)));
            }
        }

        public static enum Level {
            Level1(Meters.of(0.592953), Degrees.of(35), Meters.of(0.779254)),
            Level2(Meters.of(0.792953), Degrees.of(35), Meters.of(0.779254), new Transform2d(
                new Translation2d(
                    Inches.of(20),
                    Inches.of(-5.5)
                ),
                new Rotation2d(Degrees.of(-20))
            ).inverse()),
            Level3(Meters.of(1.196053), Degrees.of(35), Meters.of(0.779254), new Transform2d(
                new Translation2d(
                    Inches.of(21),
                    Inches.of(-5.5)
                ),
                new Rotation2d(Degrees.of(-20))
            ).inverse()),
            Level4(Meters.of(1.828663), Degrees.of(90), Meters.of(0.780750), new Transform2d(
                new Translation2d(
                    Inches.of(22),
                    Inches.of(-2.5)
                ),
                new Rotation2d(Degrees.of(-60))
            ).inverse()),
            ;
            private final Transform3d transform;
            public final RobotFlippedSuperstructureState superstructureStates;
            Level(Distance height, Angle angle, Distance radius) {
                this(height, angle, radius, SuperstructureConstants.coralScoringForwardTransform, SuperstructureState.zero, SuperstructureState.zero);
            }
            Level(Distance height, Angle angle, Distance radius, Transform2d forwardOffsetRobotSpace) {
                this(height, angle, radius, forwardOffsetRobotSpace, SuperstructureState.zero, SuperstructureState.zero);
            }
            Level(Distance height, Angle angle, Distance radius, Transform2d forwardOffsetRobotSpace, SuperstructureState forwardOffset, SuperstructureState backwardOffset) {
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
                this.superstructureStates = RobotFlippedSuperstructureState.fromForwardRobotFlipped(SuperstructureState.fromRobotSpace(
                    new Pose2d(
                        new Translation2d(
                            RobotConstants.centerToFrontBumper.plus(minimumReefRadius).minus(radius),
                            height
                        ),
                        new Rotation2d(
                            angle.unaryMinus()
                        )
                    )
                    .transformBy(forwardOffsetRobotSpace)
                ).plus(backwardOffset));
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

        public static final class Pipe {
            public final Rack rack;
            public final Side side;
            
            private final Pose3d pose;
            public final AllianceFlipped<RobotFlippedRobotPose> robotPose;

            public Branch[] branches = new Branch[Level.values().length];

            public Pipe(Rack rack, Side side) {
                this.rack = rack;
                this.side = side;
                this.pose = new Pose3d(rack.origin).transformBy(side.transform);
                this.robotPose = AllianceFlipped.fromBlue(RobotFlippedRobotPose.fromForwardRobotFlipped(rack.origin.transformBy(side.scoringTransform)));
            }

            public static int getIndex(Rack rack, Side side) {
                return rack.ordinal() * Side.values().length + side.ordinal();
            }

            public int getIndex() {
                return Pipe.getIndex(rack, side);
            }

            public char getLetter() {
                return (char) (getIndex() + 'A');
            }
        }

        public static final class Branch {
            public final Pipe pipe;
            public final Level level;

            public final AllianceFlipped<Pose3d> pose;

            public Branch(Pipe pipe, Level level) {
                this.pipe = pipe;
                this.level = level;
                this.pose = AllianceFlipped.fromBlue(pipe.pose.transformBy(level.transform));
            }

            public static int getIndex(Pipe pipe, Level level) {
                return pipe.getIndex() * Level.values().length + level.ordinal();
            }

            public static int getIndex(Rack pipe, Side side, Level level) {
                return getIndex(pipes[Pipe.getIndex(pipe, side)], level);
            }

            public int getIndex() {
                return getIndex(pipe, level);
            }
        }

        public static final Pipe[] pipes = new Pipe[Rack.values().length * Side.values().length];
        public static final Branch[] branches = new Branch[Rack.values().length * Side.values().length * Level.values().length];
        
        static {
            for (var rack : Rack.values()) {
                for (var side : Side.values()) {
                    pipes[Pipe.getIndex(rack, side)] =
                        rack.leftPipe.side.equals(side) ?
                            rack.leftPipe :
                            rack.rightPipe
                    ;
                }
            }

            for (var pipe : pipes) {
                for (var level : Level.values()) {
                    var branch = new Branch(pipe, level);
                    pipe.branches[level.ordinal()] = branch;
                    branches[Branch.getIndex(pipe, level)] = branch;
                }
            }
        }

        public static final Pipe getPipe(Rack rack, Side side) {
            return pipes[Pipe.getIndex(rack, side)];
        }

        public static final Pipe getPipe(int rack, int side) {
            return getPipe(Rack.values()[rack], Side.values()[side]);
        }

        public static final Branch getBranch(Pipe pipe, Level level) {
            return branches[Branch.getIndex(pipe, level)];
        }

        public static final Branch getBranch(Rack rack, Side side, Level level) {
            return branches[Branch.getIndex(rack, side, level)];
        }

        public static final Branch getBranch(int pipe, int level) {
            return getBranch(pipes[pipe], Level.values()[level]);
        }

        public static final Branch getBranch(int rack, int side, int level) {
            return getBranch(Rack.values()[rack], Side.values()[side], Level.values()[level]);
        }

        public static enum AlgaeLevel {
            High(Meters.of(0.679337), Meters.of(1.313180)),
            Low(Meters.of(0.679337), Meters.of(0.909320)),
            ;
            private final Transform3d transform;
            public final RobotFlippedSuperstructureState superstructurePosition;
            AlgaeLevel(Distance radius, Distance height) {
                this.transform = new Transform3d(
                    new Translation3d(
                        radius.unaryMinus(),
                        Meters.zero(),
                        height
                    ),
                    Rotation3d.kZero
                );
                this.superstructurePosition = RobotFlippedSuperstructureState.fromForwardRobotFlipped(SuperstructureState.fromRobotSpace(
                    new Pose2d(
                        new Translation2d(
                            RobotConstants.centerToFrontBumper.plus(minimumReefRadius).minus(radius),
                            height
                        ),
                        Rotation2d.kZero
                    )
                    .transformBy(SuperstructureConstants.algaeStagedForwardTransform)
                ));
            }
        }

        public static class StagedAlgae {
            public final Rack rack;
            public final AlgaeLevel algaeLevel;
            public final AllianceFlipped<Pose3d> pose;
            
            public StagedAlgae (Rack rack, AlgaeLevel algaeLevel) {
                this.rack = rack;
                this.algaeLevel = algaeLevel;
                this.pose = AllianceFlipped.fromBlue(new Pose3d(rack.origin).transformBy(algaeLevel.transform));
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
                stagedAlgae[StagedAlgae.getIndex(rack)] = rack.stagedAlgae;
            }
        }

        public static StagedAlgae getStagedAlgae(Rack rack) {
            return stagedAlgae[rack.ordinal()];
        }
    
        public static StagedAlgae getStagedAlgae(int rack) {
            return getStagedAlgae(Rack.values()[rack]);
        }
    }

    public static final class Processor {
        public static final AllianceFlipped<Pose2d> processorTargetPose = AllianceFlipped.fromBlue(new Pose2d(
            new Translation2d(
                Meters.of(6.057646),
                Algae.radius.times(2).plus(RobotConstants.centerToFrontBumper).plus(Inches.of(6))
            ),
            Rotation2d.kCW_90deg
        ));

        public static final SuperstructureState superstructureState = SuperstructureState.fromParts(
            PivotConstants.minAngle,
            ElevatorConstants.minLengthPhysical,
            Degrees.of(-10)
        );
    }

    public static final class Barge {
        private final static Pose2d bargeMidpoint = new Pose2d(
            new Translation2d(
                Meters.of(8.774113),
                Meters.of(6.130925)
            ),
            Rotation2d.kZero
        );
        private final static Transform2d bargeRightScoringTransform = new Transform2d(
            new Translation2d(
                Meters.of(0.568325).plus(RobotConstants.centerToFrontBumper).unaryMinus(),
                Meters.of(10.90600).unaryMinus()
            ),
            Rotation2d.kZero
        );
        private final static Transform2d bargeCenterScoringTransform = new Transform2d(
            new Translation2d(
                Meters.of(0.568325).plus(RobotConstants.centerToFrontBumper).unaryMinus(),
                Meters.zero()
            ),
            Rotation2d.kZero
        );
        private final static Transform2d bargeLeftScoringTransform = new Transform2d(
            new Translation2d(
                Meters.of(0.568325).plus(RobotConstants.centerToFrontBumper).unaryMinus(),
                Meters.of(1.0907522)
            ),
            Rotation2d.kZero
        );

        public static final AllianceFlipped<Pose2d> rightBargePose = AllianceFlipped.fromBlue(bargeMidpoint.transformBy(bargeRightScoringTransform));
        public static final AllianceFlipped<Pose2d> centerBargePose = AllianceFlipped.fromBlue(bargeMidpoint.transformBy(bargeCenterScoringTransform));
        public static final AllianceFlipped<Pose2d> leftBargePose = AllianceFlipped.fromBlue(bargeMidpoint.transformBy(bargeLeftScoringTransform));

        // TODO
        public static final RobotFlippedSuperstructureState superstructurePosition = new RobotFlippedSuperstructureState(
            SuperstructureState.fromRobotSpace(
                new Pose2d(
                    new Translation2d(
                        RobotConstants.centerToFrontBumper,
                        Meters.of(2.26200)
                    ),
                    Rotation2d.kZero
                )
                .transformBy(SuperstructureConstants.algaeOuttakeForwardTransform)
            ),
            SuperstructureState.fromParts(
                Degrees.of(0),
                ElevatorConstants.maxLengthPhysical,
                Degrees.of(0)
            )
        );

        public static enum Cage {
            LeftCage(Meters.of(5.0784252)),
            MiddleCage(Meters.of(6.169025)),
            RightCage(Meters.of(7.2596248))
            ;
            public final AllianceFlipped<Pose2d> robotPose;
            private final Translation2d transform;
            private final Transform2d scoringTransform;
            private final Distance minimumScoringDistance = Meters.of(0.06809);
            Cage (Distance yDistance) {
                this.transform = new Translation2d(
                    Meters.of(8.7741252),
                    yDistance
                );
                this.scoringTransform = new Transform2d(
                    new Translation2d(
                        RobotConstants.centerToFrontBumper.plus(minimumScoringDistance).plus(ClimberConstants.climberBackwardOffset).unaryMinus(),
                        Meters.zero()
                    ),
                    Rotation2d.k180deg
                );
                robotPose = AllianceFlipped.fromBlue(new Pose2d(transform, Rotation2d.kZero).transformBy(scoringTransform), FieldFlipType.XenterLineMirror);
            }
        }
    
    }
}