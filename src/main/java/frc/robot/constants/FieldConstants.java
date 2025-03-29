package frc.robot.constants;

import static edu.wpi.first.units.Units.Centimeters;
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
import frc.robot.subsystems.superstructure.Superstructure.RobotFlippedTotalState;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.pivot.PivotConstants;
import frc.util.flipping.AllianceFlipUtil;
import frc.util.flipping.AllianceFlipUtil.FieldFlipType;
import frc.util.flipping.AllianceFlippable;
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
            SuperstructureState.fromWristAxisRobotSpace(
                new Pose2d(
                    new Translation2d(
                        RobotConstants.centerToFrontBumper.minus(Inches.of(3)),//.plus(Coral.radius.times(2)),
                        chuteBottomHeight.plus(Coral.radius.times(Math.cos(chuteAngle.in(Radians)))).plus(Inches.of(-2.75))
                    ),
                    new Rotation2d(chuteAngle)
                )
                .transformBy(SuperstructureConstants.coralIntakeForwardTransform)
            ),
            SuperstructureState.fromParts(
                Degrees.of(85),
                ElevatorConstants.minLengthPhysical,
                Degrees.of(148)
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

        public static enum Side {
            Left(Meters.of(+0.164309)),
            Right(Meters.of(-0.164309)),
            ;
            private final Transform2d pipeCenterTransform;
            Side(Distance yOffset) {
                this.pipeCenterTransform = new Transform2d(
                    new Translation2d(
                        Meters.of(0.526750).unaryMinus(),
                        yOffset
                    ),
                    Rotation2d.kZero
                );
            }
        }

        public static final AllianceFlipped<ReefObject> reefs = AllianceFlipped.fromBlue(new ReefObject(new Pose2d(reefCenter.getBlue(), Rotation2d.kZero)));

        public static final class ReefObject implements AllianceFlippable<ReefObject> {
            public final Pose2d reefCenter;

            public final RackObject[] racks = new RackObject[6];
            public final StagedAlgaeObject[] stagedAlgae = new StagedAlgaeObject[6];
            public final PipeObject[] pipes = new PipeObject[12];
            public final BranchObject[] branches = new BranchObject[36];

            private ReefObject(Pose2d reefCenter) {
                this.reefCenter = reefCenter;

                var rackDelta = Rotation2d.fromDegrees(60);
                for (int rackIndex = 0; rackIndex < 6; rackIndex++) {
                    var rack = new RackObject(
                        new Pose2d(
                            reefCenter.getTranslation(),
                            reefCenter.getRotation().plus(rackDelta.times(rackIndex))
                        ),
                        rackIndex
                    );
                    racks[rackIndex] = rack;

                    for (var side : Side.values()) {
                        var pipe = new PipeObject(rack, side);
                        pipes[rackIndex * 2 + side.ordinal()] = pipe;

                        for (var level : BranchLevel.values()) {
                            var branch = new BranchObject(pipe, level);
                            branches[level.ordinal() * 12 + rackIndex * 2 + side.ordinal()] = branch;
                        }
                    }

                    stagedAlgae[rackIndex] = new StagedAlgaeObject(rack, StagedAlgaeLevel.values()[rackIndex % 2]);
                }
            }

            @Override
            public ReefObject flip(FieldFlipType flipType) {
                return new ReefObject(AllianceFlipUtil.flip(this.reefCenter, flipType));
            }
        }

        public static final class RackObject {
            public static final Transform2d robotTransform = new Transform2d(
                new Translation2d(
                    minimumReefRadius.plus(RobotConstants.centerToFrontBumper).unaryMinus(),
                    Meters.zero()
                ),
                Rotation2d.kZero
            );

            public final int id;

            public final Pose2d intersectionPose;
            public final RobotFlippedRobotPose centerRobotPose;

            private RackObject(Pose2d intersectionPose, int id) {
                this.intersectionPose = intersectionPose;
                this.centerRobotPose = RobotFlippedRobotPose.fromForwardRobotFlipped(this.intersectionPose.transformBy(robotTransform));
                this.id = id;
            }
        }
        public static final class RackConcept extends AllianceFlipped<RackObject> {
            public final int id;

            private RackConcept(int id, AllianceFlipped<RackObject> rackObjects) {
                super(rackObjects.getBlue(), rackObjects.getRed());
                this.id = id;
            }
        }

        public static final class PipeObject {
            public final RackObject rack;
            public final Side side;
            public final int id;

            public final Pose2d pipeCenterPose;
            public final RobotFlippedRobotPose robotPose;

            private PipeObject(RackObject rack, Side side) {
                this.rack = rack;
                this.side = side;
                this.id = this.rack.id * 2 + this.side.ordinal();

                this.pipeCenterPose = this.rack.intersectionPose.transformBy(this.side.pipeCenterTransform);
                this.robotPose = RobotFlippedRobotPose.fromForwardRobotFlipped(this.pipeCenterPose.transformBy(new Transform2d(
                    new Translation2d(
                        Meters.of(0.304987).plus(RobotConstants.centerToFrontBumper).unaryMinus(),
                        Meters.zero()
                    ),
                    Rotation2d.kZero
                )));
            }

            public String getLetter() {
                return Character.toString('A' + id);
            }
        }
        public static final class PipeConcept extends AllianceFlipped<PipeObject> {
            public final RackConcept rack;
            public final Side side;
            public final int id;

            private PipeConcept(RackConcept rack, Side side, AllianceFlipped<PipeObject> pipeObjects) {
                super(pipeObjects.getBlue(), pipeObjects.getRed());
                this.rack = rack;
                this.side = side;
                this.id = this.rack.id * 2 + this.side.ordinal();
            }

            public String getLetter() {
                return Character.toString('A' + id);
            }
        }

        public static enum BranchLevel {
            Level2(Meters.of(0.792953), Meters.of(0.252504), Degrees.of(35),
                RobotFlippedSuperstructureState.fromForwardOnly(
                    SuperstructureState.fromCoralTipRobotSpace(
                        new Pose2d(
                            new Translation2d(
                                RobotConstants.centerToFrontBumper.plus(Inches.of(1)),
                                Meters.of(0.792953).plus(Inches.of(5))
                            ),
                            Rotation2d.fromDegrees(-15)
                        )
                    )
                )
            ),
            Level3(Meters.of(1.196053), Meters.of(0.252504), Degrees.of(35),
                RobotFlippedSuperstructureState.fromForwardOnly(
                    SuperstructureState.fromCoralTipRobotSpace(
                        new Pose2d(
                            new Translation2d(
                                RobotConstants.centerToFrontBumper.plus(Inches.of(1)),
                                Meters.of(1.196053).plus(Inches.of(5))
                            ),
                            Rotation2d.fromDegrees(-15)
                        )
                    )
                )
            ),
            Level4(Meters.of(1.828663), Meters.of(0.254000), Degrees.of(90),
                RobotFlippedSuperstructureState.fromForwardOnly(
                    SuperstructureState.fromCoralTipRobotSpace(
                        new Pose2d(
                            new Translation2d(
                                RobotConstants.centerToFrontBumper.plus(Inches.of(2)),
                                Meters.of(1.828663).plus(Inches.of(3))
                            ),
                            Rotation2d.fromDegrees(-30)
                        )
                    )
                )
            ),
            ;
            private final Transform3d branchTipTransform;

            public final RobotFlippedSuperstructureState scoringSuperstructureStates;
            BranchLevel(Distance branchTipHeight, Distance branchTipHorizontalLength, Angle branchTipAngle, RobotFlippedSuperstructureState scoringSuperstructureStates) {
                this.branchTipTransform = new Transform3d(
                    new Translation3d(
                        branchTipHorizontalLength.unaryMinus(),
                        Meters.zero(),
                        branchTipHeight
                    ),
                    new Rotation3d(
                        Degrees.zero(),
                        branchTipAngle,
                        Degrees.zero()
                    )
                );
                this.scoringSuperstructureStates = scoringSuperstructureStates;
            }
        }
        
        public static final class BranchObject {
            public final PipeObject pipe;
            public final BranchLevel level;
            public final int id;
            
            public final Pose3d branchTip;
            public final RobotFlippedTotalState scoreTotalState;

            private BranchObject(PipeObject pipe, BranchLevel level) {
                this.pipe = pipe;
                this.level = level;
                this.id = this.level.ordinal() * 12 + this.pipe.id;
                this.branchTip = new Pose3d(this.pipe.pipeCenterPose).transformBy(this.level.branchTipTransform);
                this.scoreTotalState = RobotFlippedTotalState.combine(this.pipe.robotPose, this.level.scoringSuperstructureStates);
            }
        }
        public static final class BranchConcept extends AllianceFlipped<BranchObject> {
            public final PipeConcept pipe;
            public final BranchLevel level;
            public final int id;
            
            private BranchConcept(PipeConcept pipe, BranchLevel level, AllianceFlipped<BranchObject> branchObjects) {
                super(branchObjects.getBlue(), branchObjects.getRed());
                this.pipe = pipe;
                this.level = level;
                this.id = this.level.ordinal() * 12 + this.pipe.id;
            }
        }

        public static enum StagedAlgaeLevel {
            High(Meters.of(1.313180), Meters.of(0.679337), RobotFlippedSuperstructureState.fromForwardOnly(
                SuperstructureState.fromAlgaeCenterRobotSpace(
                    new Pose2d(
                        new Translation2d(
                            Meters.of(0.679337),
                            Meters.of(1.313180)
                        ),
                        Rotation2d.fromDegrees(-15)
                    )
                )
            )),
            Low(Meters.of(0.909320), Meters.of(0.679337), RobotFlippedSuperstructureState.fromForwardOnly(
                SuperstructureState.fromAlgaeCenterRobotSpace(
                    new Pose2d(
                        new Translation2d(
                            Meters.of(0.679337),
                            Meters.of(0.909320)
                        ),
                        Rotation2d.fromDegrees(-15)
                    )
                )
            )),
            ;
            public final Transform3d transform;
            public final RobotFlippedSuperstructureState intakeSuperstructureStates;
            StagedAlgaeLevel(Distance height, Distance radius, RobotFlippedSuperstructureState intakeSuperstructureStates) {
                this.transform = new Transform3d(
                    new Translation3d(
                        radius.unaryMinus(),
                        Meters.zero(),
                        height
                    ),
                    Rotation3d.kZero
                );
                this.intakeSuperstructureStates = intakeSuperstructureStates;
            }
        }

        public static final class StagedAlgaeObject {
            public final RackObject rack;
            public final StagedAlgaeLevel level;

            public final Pose3d centerPose;
            public final RobotFlippedTotalState intakeTotalState;

            private StagedAlgaeObject(RackObject rack, StagedAlgaeLevel level) {
                this.rack = rack;
                this.level = level;

                this.centerPose = new Pose3d(this.rack.intersectionPose).transformBy(this.level.transform);
                this.intakeTotalState = RobotFlippedTotalState.combine(this.rack.centerRobotPose, this.level.intakeSuperstructureStates);
            }
        }
        public static final class StagedAlgaeConcept extends AllianceFlipped<StagedAlgaeObject> {
            public final RackConcept rack;
            public final StagedAlgaeLevel level;

            private StagedAlgaeConcept(RackConcept rack, StagedAlgaeLevel level, AllianceFlipped<StagedAlgaeObject> algaeObjects) {
                super(algaeObjects.getBlue(), algaeObjects.getRed());
                this.rack = rack;
                this.level = level;
            }
        }

        public static final RackConcept[] racks;
        public static final PipeConcept[] pipes;
        public static final BranchConcept[] branches;
        public static final StagedAlgaeConcept[] stagedAlgae;

        static {
            racks = new RackConcept[6];
            pipes = new PipeConcept[12];
            branches = new BranchConcept[36];
            stagedAlgae = new StagedAlgaeConcept[6];
            for (int rackIndex = 0; rackIndex < 6; rackIndex++) {
                var rack = new RackConcept(rackIndex, new AllianceFlipped<RackObject>(reefs.getBlue().racks[rackIndex], reefs.getRed().racks[rackIndex]));
                racks[rackIndex] = rack;

                for (var side : Side.values()) {
                    var pipeIndex = rackIndex * 2 + side.ordinal();
                    var pipe = new PipeConcept(rack, side, new AllianceFlipped<PipeObject>(reefs.getBlue().pipes[pipeIndex], reefs.getRed().pipes[pipeIndex]));
                    pipes[pipeIndex] = pipe;

                    for (var level : BranchLevel.values()) {
                        var branchIndex = level.ordinal() * 12 + pipe.id;
                        var branch = new BranchConcept(pipe, level, new AllianceFlipped<BranchObject>(reefs.getBlue().branches[branchIndex], reefs.getRed().branches[branchIndex]));
                        branches[branchIndex] = branch;
                    }
                }

                stagedAlgae[rackIndex] = new StagedAlgaeConcept(rack, StagedAlgaeLevel.values()[rackIndex % 2], new AllianceFlipped<StagedAlgaeObject>(reefs.getBlue().stagedAlgae[rackIndex], reefs.getRed().stagedAlgae[rackIndex]));
            }
        }
    }

    public static final class Processor {
        public static final AllianceFlipped<RobotFlippedRobotPose> processorTargetPose = AllianceFlipped.fromBlue(RobotFlippedRobotPose.fromForwardOnly(new Pose2d(
            new Translation2d(
                Meters.of(6.057646),
                Algae.radius.times(2).plus(RobotConstants.centerToFrontBumper).plus(Inches.of(6))
            ),
            Rotation2d.kCW_90deg
        )));

        public static final RobotFlippedSuperstructureState superstructureState = RobotFlippedSuperstructureState.fromForwardOnly(SuperstructureState.fromParts(
            PivotConstants.minAngle,
            ElevatorConstants.minLengthPhysical,
            Degrees.of(10)
        ));
    }

    public static final class Barge {
        private final static Pose2d bargeMidpoint = new Pose2d(
            new Translation2d(
                Meters.of(8.774113),
                Meters.of(6.130925).minus(Meters.of(1.0907522).div(2))
            ),
            Rotation2d.kZero
        );
        private final static Transform2d bargeRightScoringTransform = new Transform2d(
            new Translation2d(
                Inches.of(6).plus(RobotConstants.centerToFrontBumper).unaryMinus(),
                Meters.of(1.0907522).unaryMinus()
            ),
            Rotation2d.kZero
        );
        private final static Transform2d bargeCenterScoringTransform = new Transform2d(
            new Translation2d(
                Inches.of(6).plus(RobotConstants.centerToFrontBumper).unaryMinus(),
                Meters.zero()
            ),
            Rotation2d.kZero
        );
        private final static Transform2d bargeLeftScoringTransform = new Transform2d(
            new Translation2d(
                Inches.of(6).plus(RobotConstants.centerToFrontBumper).unaryMinus(),
                Meters.of(1.0907522)
            ),
            Rotation2d.kZero
        );

        public static final AllianceFlipped<RobotFlippedRobotPose> rightBargePose = AllianceFlipped.fromBlue(RobotFlippedRobotPose.fromForwardPivotFlipped(bargeMidpoint.transformBy(bargeRightScoringTransform)));
        public static final AllianceFlipped<RobotFlippedRobotPose> centerBargePose = AllianceFlipped.fromBlue(RobotFlippedRobotPose.fromForwardPivotFlipped(bargeMidpoint.transformBy(bargeCenterScoringTransform)));
        public static final AllianceFlipped<RobotFlippedRobotPose> leftBargePose = AllianceFlipped.fromBlue(RobotFlippedRobotPose.fromForwardPivotFlipped(bargeMidpoint.transformBy(bargeLeftScoringTransform)));

        public static final RobotFlippedSuperstructureState superstructureState = RobotFlippedSuperstructureState.fromForwardPivotFlipped(
            SuperstructureState.fromParts(
                Degrees.of(90),
                ElevatorConstants.maxLengthSoftware,
                Degrees.of(60)
            )
        );

        public static enum Cage {
            InnerCage(Meters.of(5.0784252)),
            MiddleCage(Meters.of(6.169025)),
            OuterCage(Meters.of(7.2596248))
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