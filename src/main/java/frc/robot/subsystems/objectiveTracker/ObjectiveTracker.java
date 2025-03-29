package frc.robot.subsystems.objectiveTracker;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.FieldConstants.Barge;
import frc.robot.constants.FieldConstants.Coral;
import frc.robot.constants.FieldConstants.CoralStation;
import frc.robot.constants.FieldConstants.Processor;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Reef.BranchConcept;
import frc.robot.constants.FieldConstants.Reef.BranchLevel;
import frc.robot.constants.FieldConstants.Reef.BranchObject;
import frc.robot.constants.FieldConstants.Reef.RackObject;
import frc.robot.constants.FieldConstants.Reef.StagedAlgaeConcept;
import frc.robot.constants.FieldConstants.Reef.StagedAlgaeLevel;
import frc.robot.constants.FieldConstants.Reef.StagedAlgaeObject;
import frc.robot.subsystems.superstructure.Superstructure.Direction;
import frc.robot.subsystems.superstructure.Superstructure.RobotFlippedRobotPose;
import frc.robot.subsystems.superstructure.Superstructure.RobotFlippedSuperstructureState;
import frc.robot.subsystems.superstructure.Superstructure.RobotFlippedTotalState;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.util.VirtualSubsystem;
import frc.util.loggerUtil.LoggerUtil;

public class ObjectiveTracker extends VirtualSubsystem {
    private final ObjectiveSelectorIO io;
    private final ObjectiveSelectorIOInputsAutoLogged inputs = new ObjectiveSelectorIOInputsAutoLogged();

    public static enum AlgaeGoal {
        NET,
        PROCESSOR,
        OPPONENT_PROCESSOR,
        ;
    }

    public static enum Priority {
        Level4Fill(Optional.of(BranchLevel.Level4)) {
            @Override
            public boolean completed(boolean[] branchStates) {
                for (int i = 24; i < 36; i++) {
                    if (branchStates[i] == false) return false;
                }
                return true;
            }
        },
        Level3Fill(Optional.of(BranchLevel.Level3)) {
            @Override
            public boolean completed(boolean[] branchStates) {
                for (int i = 12; i < 24; i++) {
                    if (branchStates[i] == false) return false;
                }
                return true;
            }
        },
        Level2Fill(Optional.of(BranchLevel.Level2)) {
            @Override
            public boolean completed(boolean[] branchStates) {
                for (int i = 0; i < 12; i++) {
                    if (branchStates[i] == false) return false;
                }
                return true;
            }
        },
        Level1Fill(Optional.empty()) {
            @Override
            public boolean completed(boolean[] branchStates) {
                return false;
            }
        },
        Level4RP(Optional.of(BranchLevel.Level4)) {
            @Override
            public boolean completed(boolean[] branchStates) {
                var count = 0;
                for (int i = 24; i < 36; i++) {
                    if (branchStates[i] == true) count++;
                    if (count >= 5) return true;
                }
                return false;
            }
        },
        Level3RP(Optional.of(BranchLevel.Level3)) {
            @Override
            public boolean completed(boolean[] branchStates) {
                var count = 0;
                for (int i = 12; i < 24; i++) {
                    if (branchStates[i] == true) count++;
                    if (count >= 5) return true;
                }
                return false;
            }
        },
        Level2RP(Optional.of(BranchLevel.Level2)) {
            @Override
            public boolean completed(boolean[] branchStates) {
                var count = 0;
                for (int i = 0; i < 12; i++) {
                    if (branchStates[i] == true) count++;
                    if (count >= 5) return true;
                }
                return false;
            }
        },
        Level1RP(Optional.empty()) {
            @Override
            public boolean completed(boolean[] branchStates) {
                return false; //TODO: Level 1 count
            }
        },
        ;
        public final Optional<BranchLevel> level;
        Priority(Optional<BranchLevel> level) {
            this.level = level;
        }
        public boolean completed(boolean[] branchStates) {
            return false;
        }
    }

    private AlgaeGoal selectedAlgaeGoal = AlgaeGoal.NET;

    private final boolean[] branchStates = new boolean[] {
        false,false,true,false,true,true,true,true,true,true,true,true,
        true,true,true,true,true,true,true,false,true,false,true,true,
        true,true,true,false,true,true,true,true,true,true,true,true,
    };
    private final boolean[] algaeStates = new boolean[] {true,true,true,false,false,false};

    private final Set<BranchConcept> availableBranches = new HashSet<>(36);
    private final Set<BranchConcept> availableUnblockedBranches = new HashSet<>(36);

    private final Set<StagedAlgaeConcept> availableAlgae = new HashSet<>(6);

    private final List<Priority> fullStrategy = List.of(Priority.values());
    private final List<Priority> uncompletedPriorities = List.of(
        Priority.Level4RP,
        Priority.Level3RP,
        Priority.Level2RP,
        Priority.Level1RP,
        Priority.Level4Fill,
        Priority.Level3Fill,
        Priority.Level2Fill,
        Priority.Level1Fill
    );

    public static enum ObjectiveType {
        IntakeCoral(false),
        IntakeAlgae(true),
        ScoreCoral(true),
        ScoreAlgae(false),
        ;
        public final boolean isReefObjective;
        ObjectiveType(boolean isReefObjective) {
            this.isReefObjective = isReefObjective;
        }
    }

    private NonReefObjective intakeCoralObjective;
    private Optional<IntakeAlgaeObjective> intakeAlgaeObjective;
    private ScoreCoralObjective scoreCoralObjective;
    private ScoreAlgaeObjective scoreAlgaeObjective;
    private Optional<Objective> target;
    private Optional<ObjectiveType> typeOverride = Optional.empty();

    public ObjectiveTracker(ObjectiveSelectorIO io) {
        System.out.println("[Init ObjectiveTracker] Instantiating ObjectiveTracker with " + io.getClass().getSimpleName());
        this.io = io;

        updateBranches();
        updateAlgae();
        updateUnblockedBranches();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Objective Tracker", inputs);

        // if (inputs.coral != -1) {
        //     var rack = inputs.coral >> 3 & 0b1111;
        //     var side = inputs.coral >> 2 & 0b1;
        //     var level = inputs.coral & 0b11;
        //     selectedBranch = FieldConstants.Reef.getBranch(rack, side, level);
        //     inputs.coral = -1;
        // }
        if (inputs.algae != -1) {
            selectedAlgaeGoal = AlgaeGoal.values()[inputs.algae];
            inputs.algae = -1;
        }
        // if (inputs.intake != -1) {
        //     selectedIntakeGoal = 
        //         inputs.intake == 0 ?
        //         Optional.empty() :
        //         inputs.intake == 1 ?
        //         Optional.of(Optional.empty()) :
        //         Optional.of(Optional.of(FieldConstants.Reef.stagedAlgae[inputs.intake - 2]));
        //     inputs.intake = -1;
        // }

        // io.setCoral(
        //     selectedBranch.pipe.rack.ordinal() << 3 |
        //     selectedBranch.pipe.side.ordinal() << 2 |
        //     selectedBranch.level.ordinal()
        // );
        io.setAlgae(selectedAlgaeGoal.ordinal());
        // io.setIntake(
        //     selectedIntakeGoal.isEmpty() ? 0 :
        //     selectedIntakeGoal.get().isEmpty() ? 1 :
        //     selectedIntakeGoal.get().get().getIndex() + 2
        // );

        // Logger.recordOutput("Objective Tracker/Selected Branch", selectedBranch.pose.getOurs());

        var branchesChanged = false;
        for (var changedBranch : inputs.branchQueue) {
            branchesChanged = true;
            var branchState = changedBranch >= 0;
            var branchID = branchState ? changedBranch : changedBranch + 37;
            branchStates[branchID] = branchState;
        }
        var algaeChanged = false;
        for (var changedBranch : inputs.toggledAlgae) {
            algaeChanged = true;
            var algaeState = changedBranch >= 0;
            var algaeID = algaeState ? changedBranch : changedBranch + 37;
            algaeStates[algaeID] = algaeState;
        }

        if (branchesChanged) {
            updateBranches();
        }

        if (algaeChanged) {
            updateAlgae();
        }

        if (algaeChanged || branchesChanged) {
            updateUnblockedBranches();
        }
    }

    private void updateBranches() {
        availableBranches.clear();
        var filledBranches = new ArrayList<Pose3d>(36);
        for (int i = 0; i < branchStates.length; i++) {
            if (branchStates[i] == false) {
                availableBranches.add(Reef.branches[i]);
            } else {
                filledBranches.add(Reef.reefs.getOurs().branches[i].branchTip.transformBy(Coral.branchPlacement));
            }
        }
        Logger.recordOutput("Objective Tracker/Reef/Coral", filledBranches.toArray(Pose3d[]::new));
    }
    private void updateAlgae() {
        availableAlgae.clear();
        var filledAlgae = new ArrayList<Pose3d>(36);
        for (int i = 0; i < algaeStates.length; i++) {
            if (algaeStates[i] == true) {
                availableAlgae.add(Reef.stagedAlgae[i]);
                filledAlgae.add(Reef.reefs.getOurs().stagedAlgae[i].centerPose);
            }
        }
        Logger.recordOutput("Objective Tracker/Reef/Algae", filledAlgae.toArray(Pose3d[]::new));
    }
    private void updateUnblockedBranches() {
        availableUnblockedBranches.clear();
        for (var availableBranch : availableBranches) {
            var blocked = false;
            if (availableBranch.level != BranchLevel.Level4) {
                for (var stagedAlgae : availableAlgae) {
                    if (stagedAlgae.rack != availableBranch.pipe.rack) continue;
                    if (stagedAlgae.level == StagedAlgaeLevel.High && availableBranch.level == BranchLevel.Level2) continue;
                    blocked = true;
                    break;
                }
            }
            if (!blocked) {
                availableUnblockedBranches.add(availableBranch);
            }
        }
    }

    public void determineGoal(Pose2d currentPose, boolean hasCoral, boolean hasAlgae) {
        var stationPoses = new RobotFlippedRobotPose[] {
            CoralStation.leftStationLeft.getOurs(),
            CoralStation.leftStationCenter.getOurs(),
            CoralStation.leftStationRight.getOurs(),
            CoralStation.rightStationLeft.getOurs(),
            CoralStation.rightStationCenter.getOurs(),
            CoralStation.rightStationRight.getOurs(),
        };
        var closestStationPose = Arrays.stream(stationPoses).sorted((a,b) -> {
            var aDistance = a.getClosest(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
            var bDistance = b.getClosest(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
            return (int) Math.signum(aDistance - bDistance);
        }).findFirst().get();
        this.intakeCoralObjective = NonReefObjective.fromParts(closestStationPose, CoralStation.intakePosition, currentPose.getRotation(), ObjectiveType.IntakeCoral);

        var closestRacks = Arrays.stream(Reef.reefs.getOurs().racks)
            .sorted((a,b) -> {
                var aDistance = a.centerRobotPose.getClosest(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
                var bDistance = b.centerRobotPose.getClosest(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
                return (int) Math.signum(aDistance - bDistance);
            })
            .limit(3)
            .toList()
        ;

        var closestAlgae = availableAlgae.stream().map((algae) -> algae.getOurs())
            .filter((algae) -> closestRacks.contains(algae.rack))
            .sorted((a,b) -> {
                var aDistance = a.intakeTotalState.getClosestRobotPose(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
                var bDistance = b.intakeTotalState.getClosestRobotPose(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
                return (int) Math.signum(aDistance - bDistance);
            })
            .findFirst()
        ;
        this.intakeAlgaeObjective = closestAlgae.map((algae) -> new IntakeAlgaeObjective(algae, currentPose.getRotation()));

        var closestPipes = Arrays.stream(Reef.reefs.getOurs().pipes)
            .sorted((a,b) -> {
                var aDistance = a.robotPose.getClosest(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
                var bDistance = b.robotPose.getClosest(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
                return (int) Math.signum(aDistance - bDistance);
            })
            .limit(6)
            .toList()
        ;
        var targetBranch = availableUnblockedBranches.stream()
            .map((branch) -> branch.getOurs())
            .filter((branch) -> closestPipes.contains(branch.pipe))
            .sorted((a,b) -> {
                if (a.level == b.level) {
                    var aDistance = a.scoreTotalState.getClosestRobotPose(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
                    var bDistance = b.scoreTotalState.getClosestRobotPose(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
                    return (int) Math.signum(aDistance - bDistance);
                } else {
                    for (var priority : uncompletedPriorities) {
                        if (priority.level.isEmpty()) continue;
                        if (a.level == priority.level.get()) return -1;
                        if (b.level == priority.level.get()) return 1;
                    }
                    return 0;
                }
            })
            .findFirst()
        ;
        if (uncompletedPriorities.get(0).level.isEmpty() || targetBranch.isEmpty()) {
            this.scoreCoralObjective = ScoreCoralObjective.fromLevel1(closestRacks.get(0));
        } else {
            this.scoreCoralObjective = ScoreCoralObjective.fromBranch(targetBranch.get(), currentPose.getRotation());
        }

        this.scoreAlgaeObjective = new ScoreAlgaeObjective(selectedAlgaeGoal, currentPose);

        Logger.recordOutput("Objective Tracker/Intake/Coral/Target Direction", intakeCoralObjective.getTargetDirection());
        Logger.recordOutput("Objective Tracker/Intake/Coral/Target Pose", intakeCoralObjective.getTargetPose());
        Logger.recordOutput("Objective Tracker/Intake/Coral/Target Mechs", intakeCoralObjective.getTargetState().getMechTransforms());
        Logger.recordOutput("Objective Tracker/Intake/Algae/Target Direction", LoggerUtil.toArray(intakeAlgaeObjective.map(IntakeAlgaeObjective::getTargetDirection), Direction[]::new));
        Logger.recordOutput("Objective Tracker/Intake/Algae/Target Pose", LoggerUtil.toArray(intakeAlgaeObjective.map(IntakeAlgaeObjective::getTargetPose), Pose2d[]::new));
        Logger.recordOutput("Objective Tracker/Intake/Algae/Target Mechs", intakeAlgaeObjective.isPresent() ? intakeAlgaeObjective.get().getTargetState().getMechTransforms() : new Transform3d[0]);
        Logger.recordOutput("Objective Tracker/Score/Coral/Target Direction", scoreCoralObjective.getTargetDirection());
        Logger.recordOutput("Objective Tracker/Score/Coral/Target Pose", scoreCoralObjective.getTargetPose());
        Logger.recordOutput("Objective Tracker/Score/Coral/Target Mechs", scoreCoralObjective.getTargetState().getMechTransforms());
        Logger.recordOutput("Objective Tracker/Score/Algae/Target Direction", scoreAlgaeObjective.getTargetDirection());
        Logger.recordOutput("Objective Tracker/Score/Algae/Target Pose", scoreAlgaeObjective.getTargetPose());
        Logger.recordOutput("Objective Tracker/Score/Algae/Target Mechs", scoreAlgaeObjective.getTargetState().getMechTransforms());

        if (typeOverride.isEmpty()) {
            if (hasCoral && hasAlgae) {
                var distToCoral = scoreCoralObjective.getTargetPose().getTranslation().getDistance(currentPose.getTranslation());
                var distToAlgae = scoreAlgaeObjective.getTargetPose().getTranslation().getDistance(currentPose.getTranslation());
                if (distToCoral < distToAlgae) {
                    target = Optional.of(scoreCoralObjective);
                } else {
                    target = Optional.of(scoreAlgaeObjective);
                }
            } else if (hasCoral) {
                target = Optional.of(scoreCoralObjective);
            } else if (hasAlgae) {
                target = Optional.of(scoreAlgaeObjective);
            } else {
                target = Optional.of(intakeCoralObjective);
            }
        } else {
            switch (typeOverride.get()) {
                default:
                case IntakeCoral:
                    target = Optional.of(intakeCoralObjective);
                break;
                case IntakeAlgae:
                    target = intakeAlgaeObjective.map((objective) -> objective);
                break;
                case ScoreCoral:
                    target = Optional.of(scoreCoralObjective);
                break;
                case ScoreAlgae:
                    target = Optional.of(scoreAlgaeObjective);
                break;
            }
        }
    }

    public NonReefObjective getIntakeCoralObjective() {
        return intakeCoralObjective;
    }
    public Optional<IntakeAlgaeObjective> getIntakeAlgaeObjective() {
        return intakeAlgaeObjective;
    }
    public ScoreCoralObjective getScoreCoralObjective() {
        return scoreCoralObjective;
    }
    public ScoreAlgaeObjective getScoreAlgaeObjective() {
        return scoreAlgaeObjective;
    }
    public Optional<Objective> getCurrentObjective() {
        return target;
    }

    public void setTypeOverride(Optional<ObjectiveType> typeOverride) {
        this.typeOverride = typeOverride;
    }

    // public void toggleSelectedBranch() {
    //     if (!placedCoral.remove(selectedBranch)) {
    //         placedCoral.add(selectedBranch);
    //     }
    // }

    public static interface Objective {
        public Pose2d getTargetPose();
        public SuperstructureState getTargetState();
        public Direction getTargetDirection();
        public ObjectiveType getObjectiveType();
    }

    public static class IntakeAlgaeObjective implements Objective {
        public final StagedAlgaeObject algae;
        private final Direction direction;

        private IntakeAlgaeObjective(StagedAlgaeObject algae, Rotation2d currentRotation) {
            this.algae = algae;
            this.direction = this.algae.intakeTotalState.getClosestDirection(currentRotation);
        }

        @Override
        public Direction getTargetDirection() {
            return direction;
        }
        @Override
        public Pose2d getTargetPose() {
            return algae.intakeTotalState.getRobotPose(direction);
        }
        @Override
        public SuperstructureState getTargetState() {
            return algae.intakeTotalState.getSuperstructureState(direction);
        }
        @Override
        public ObjectiveType getObjectiveType() {
            return ObjectiveType.IntakeAlgae;
        }
    }
    public static class ScoreCoralObjective implements Objective {
        public final Optional<BranchLevel> branchLevel;
        private final Pose2d targetPose;
        private final SuperstructureState targetState;
        private final Direction direction;

        private ScoreCoralObjective(Pose2d targetPose, SuperstructureState targetState, Direction direction, Optional<BranchLevel> branchLevel) {
            this.branchLevel = branchLevel;
            this.targetPose = targetPose;
            this.targetState = targetState;
            this.direction = direction;
        }
        public static ScoreCoralObjective fromBranch(BranchObject branch, Rotation2d currentRotation) {
            var direction = branch.scoreTotalState.getClosestDirection(currentRotation);
            return new ScoreCoralObjective(
                branch.scoreTotalState.getRobotPose(direction),
                branch.scoreTotalState.getSuperstructureState(direction),
                direction,
                Optional.of(branch.level)
            );
        }
        public static ScoreCoralObjective fromLevel1(RackObject rack) {
            return new ScoreCoralObjective(
                rack.centerRobotPose.getForward(),
                Reef.level1SuperstructureStates.getForward(),
                Direction.Forward,
                Optional.empty()
            );
        }
        @Override
        public Direction getTargetDirection() {
            return direction;
        }
        @Override
        public Pose2d getTargetPose() {
            return targetPose;
        }
        @Override
        public SuperstructureState getTargetState() {
            return targetState;
        }
        @Override
        public ObjectiveType getObjectiveType() {
            return ObjectiveType.ScoreCoral;
        }
    }
    public static class NonReefObjective implements Objective {
        public final Pose2d targetPose;
        public final SuperstructureState targetState;
        public final Direction targetDirection;
        public final ObjectiveType objectiveType;

        private NonReefObjective(Pose2d targetPose, SuperstructureState targetState, Direction targetDirection, ObjectiveType objectiveType) {
            this.targetPose = targetPose;
            this.targetState = targetState;
            this.targetDirection = targetDirection;
            this.objectiveType = objectiveType;
        }

        public static NonReefObjective fromTotalState(RobotFlippedTotalState totalState, Rotation2d currentRotation, ObjectiveType objectiveType) {
            var direction = totalState.getClosestDirection(currentRotation);
            return new NonReefObjective(
                totalState.getRobotPose(direction),
                totalState.getSuperstructureState(direction),
                direction,
                objectiveType
            );
        }
        public static NonReefObjective fromParts(RobotFlippedRobotPose pose, RobotFlippedSuperstructureState state, Rotation2d currentRotation, ObjectiveType objectiveType) {
            var direction = pose.getClosestDirection(currentRotation);
            return new NonReefObjective(
                pose.get(direction),
                state.get(direction),
                direction,
                objectiveType
            );
        }
        public static NonReefObjective fromRaw(Pose2d pose, SuperstructureState state, Direction direction, ObjectiveType objectiveType) {
            return new NonReefObjective(
                pose,
                state,
                direction,
                objectiveType
            );
        }

        @Override
        public Direction getTargetDirection() {
            return targetDirection;
        }
        @Override
        public Pose2d getTargetPose() {
            return targetPose;
        }
        @Override
        public SuperstructureState getTargetState() {
            return targetState;
        }
        @Override
        public ObjectiveType getObjectiveType() {
            return objectiveType;
        }

    }
    public static class ScoreAlgaeObjective implements Objective {
        public final AlgaeGoal algaeGoal;
        private final Pose2d targetPose;
        private final SuperstructureState targetState;
        private final Direction direction;

        private ScoreAlgaeObjective(AlgaeGoal algaeGoal, Pose2d currentPose) {
            this.algaeGoal = algaeGoal;

            switch (this.algaeGoal) {
                default:
                case PROCESSOR:
                    var algaeScoreTarget = Processor.processorTargetPose.getOurs();
                    this.direction = algaeScoreTarget.getClosestDirection(currentPose.getRotation());
                    this.targetPose = algaeScoreTarget.get(direction);
                    this.targetState = Processor.superstructureState.get(direction);
                    break;
                case OPPONENT_PROCESSOR:
                    var algaeScoreTarget2 = Processor.processorTargetPose.getTheirs();
                    this.direction = algaeScoreTarget2.getClosestDirection(currentPose.getRotation());
                    this.targetPose = algaeScoreTarget2.get(direction);
                    this.targetState = Processor.superstructureState.get(direction);
                break;
                case NET:
                    var bargePoses = new RobotFlippedRobotPose[] {
                        Barge.leftBargePose.getOurs(),
                        Barge.centerBargePose.getOurs(),
                        Barge.rightBargePose.getOurs(),
                    };
                    var closestBargePose = Arrays.stream(bargePoses).sorted(
                        (a,b) -> {
                            var aDistance = a.getClosest(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
                            var bDistance = b.getClosest(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
                            return (int) Math.signum(aDistance - bDistance);
                        }
                    ).findFirst().get();
                    this.direction = closestBargePose.getClosestDirection(currentPose.getRotation());
                    this.targetPose = closestBargePose.get(direction);
                    this.targetState = Barge.superstructureState.get(direction);
                break;
            }
        }

        @Override
        public Direction getTargetDirection() {
            return direction;
        }
        @Override
        public Pose2d getTargetPose() {
            return targetPose;
        }
        @Override
        public SuperstructureState getTargetState() {
            return targetState;
        }
        @Override
        public ObjectiveType getObjectiveType() {
            return ObjectiveType.ScoreAlgae;
        }
    }
}