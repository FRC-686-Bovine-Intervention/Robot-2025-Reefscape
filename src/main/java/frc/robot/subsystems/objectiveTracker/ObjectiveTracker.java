package frc.robot.subsystems.objectiveTracker;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.FieldConstants.Barge;
import frc.robot.constants.FieldConstants.CoralStation;
import frc.robot.constants.FieldConstants.Processor;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Reef.ReefObject.BranchObject;
import frc.robot.constants.FieldConstants.Reef.ReefObject.StagedAlgaeObject;
import frc.robot.subsystems.superstructure.Superstructure.Direction;
import frc.robot.subsystems.superstructure.Superstructure.RobotFlippedRobotPose;
import frc.robot.subsystems.superstructure.Superstructure.RobotFlippedSuperstructureState;
import frc.robot.subsystems.superstructure.Superstructure.RobotFlippedTotalState;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.util.VirtualSubsystem;

public class ObjectiveTracker extends VirtualSubsystem {
    private final ObjectiveSelectorIO io;
    private final ObjectiveSelectorIOInputsAutoLogged inputs = new ObjectiveSelectorIOInputsAutoLogged();

    public enum AlgaeGoal {
        NET,
        PROCESSOR,
        OPPONENT_PROCESSOR,
        ;
    }

    private AlgaeGoal selectedAlgaeGoal = AlgaeGoal.NET;

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

    private NonReefObjective intakeCoralTarget;
    private IntakeAlgaeObjective intakeAlgaeTarget;
    private ScoreCoralObjective scoreCoralTarget;
    private ScoreAlgaeObjective scoreAlgaeTarget;
    private Optional<Objective> target;
    private Optional<ObjectiveType> typeOverride;

    public ObjectiveTracker(ObjectiveSelectorIO io) {
        System.out.println("[Init] Instantiating ObjectiveTracker");
        this.io = io;
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
        this.intakeCoralTarget = NonReefObjective.fromParts(closestStationPose, CoralStation.intakePosition, currentPose.getRotation(), ObjectiveType.IntakeCoral);

        var closestAlgae = Arrays.stream(Reef.reefs.getOurs().stagedAlgae).sorted((a,b) -> {
            var aDistance = a.intakeTotalState.getClosestRobotPose(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
            var bDistance = b.intakeTotalState.getClosestRobotPose(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
            return (int) Math.signum(aDistance - bDistance);
        }).findFirst().get();
        this.intakeAlgaeTarget = new IntakeAlgaeObjective(closestAlgae, currentPose.getRotation());

        this.scoreCoralTarget = new ScoreCoralObjective(Reef.reefs.getOurs().branches[0], currentPose.getRotation());

        this.scoreAlgaeTarget = new ScoreAlgaeObjective(selectedAlgaeGoal, currentPose);

        Logger.recordOutput("Objective Tracker/Intake/Coral/Target Direction", intakeCoralTarget.getTargetDirection());
        Logger.recordOutput("Objective Tracker/Intake/Coral/Target Pose", intakeCoralTarget.getTargetPose());
        Logger.recordOutput("Objective Tracker/Intake/Coral/Target Mechs", intakeCoralTarget.getTargetState().getMechTransforms());
        Logger.recordOutput("Objective Tracker/Intake/Algae/Target Direction", intakeAlgaeTarget.getTargetDirection());
        Logger.recordOutput("Objective Tracker/Intake/Algae/Target Pose", intakeAlgaeTarget.getTargetPose());
        Logger.recordOutput("Objective Tracker/Intake/Algae/Target Mechs", intakeAlgaeTarget.getTargetState().getMechTransforms());
        Logger.recordOutput("Objective Tracker/Score/Coral/Target Direction", scoreCoralTarget.getTargetDirection());
        Logger.recordOutput("Objective Tracker/Score/Coral/Target Pose", scoreCoralTarget.getTargetPose());
        Logger.recordOutput("Objective Tracker/Score/Coral/Target Mechs", scoreCoralTarget.getTargetState().getMechTransforms());
        Logger.recordOutput("Objective Tracker/Score/Algae/Target Direction", scoreAlgaeTarget.getTargetDirection());
        Logger.recordOutput("Objective Tracker/Score/Algae/Target Pose", scoreAlgaeTarget.getTargetPose());
        Logger.recordOutput("Objective Tracker/Score/Algae/Target Mechs", scoreAlgaeTarget.getTargetState().getMechTransforms());

        if (typeOverride.isEmpty()) {
            if (hasCoral && hasAlgae) {
                var distToCoral = scoreCoralTarget.getTargetPose().getTranslation().getDistance(currentPose.getTranslation());
                var distToAlgae = scoreAlgaeTarget.getTargetPose().getTranslation().getDistance(currentPose.getTranslation());
                if (distToCoral < distToAlgae) {
                    target = Optional.of(scoreCoralTarget);
                } else {
                    target = Optional.of(scoreAlgaeTarget);
                }
            } else if (hasCoral) {
                target = Optional.of(scoreCoralTarget);
            } else if (hasAlgae) {
                target = Optional.of(scoreAlgaeTarget);
            } else {
                target = Optional.of(intakeCoralTarget);
            }
        } else {
            switch (typeOverride.get()) {
                default:
                case IntakeCoral:
                    target = Optional.of(intakeCoralTarget);
                break;
                case IntakeAlgae:
                    target = Optional.of(intakeAlgaeTarget);
                break;
                case ScoreCoral:
                    target = Optional.of(scoreCoralTarget);
                break;
                case ScoreAlgae:
                    target = Optional.of(scoreAlgaeTarget);
                break;
            }
        }
    }

    public NonReefObjective getIntakeCoralObjective() {
        return intakeCoralTarget;
    }
    public IntakeAlgaeObjective getIntakeAlgaeObjective() {
        return intakeAlgaeTarget;
    }
    public ScoreCoralObjective getScoreCoralObjective() {
        return scoreCoralTarget;
    }
    public ScoreAlgaeObjective getScoreAlgaeObjective() {
        return scoreAlgaeTarget;
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

    private BranchObject cachedBranch;
    public BranchObject getSelectedBranch() {
        return cachedBranch;
    }

    public AlgaeGoal getAlgaeGoal() {
        return selectedAlgaeGoal;
    }

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
        public final BranchObject branch;
        private final Direction direction;

        private ScoreCoralObjective(BranchObject branch, Rotation2d currentRotation) {
            this.branch = branch;
            this.direction = this.branch.scoreTotalState.getClosestDirection(currentRotation);
        }
        @Override
        public Direction getTargetDirection() {
            return direction;
        }
        @Override
        public Pose2d getTargetPose() {
            return branch.scoreTotalState.getRobotPose(direction);
        }
        @Override
        public SuperstructureState getTargetState() {
            return branch.scoreTotalState.getSuperstructureState(direction);
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