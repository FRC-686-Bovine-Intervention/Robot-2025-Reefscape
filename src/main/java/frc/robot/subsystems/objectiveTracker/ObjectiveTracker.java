package frc.robot.subsystems.objectiveTracker;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Barge;
import frc.robot.constants.FieldConstants.CoralStation;
import frc.robot.constants.FieldConstants.Processor;
import frc.robot.constants.FieldConstants.Reef.Branch;
import frc.robot.constants.FieldConstants.Reef.Level;
import frc.robot.constants.FieldConstants.Reef.Rack;
import frc.robot.constants.FieldConstants.Reef.Side;
import frc.robot.constants.FieldConstants.Reef.StagedAlgae;
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

    private final ArrayList<Branch> placedCoral = new ArrayList<>(36);
    private Branch selectedBranch = FieldConstants.Reef.branches[0];
    private AlgaeGoal selectedAlgaeGoal = AlgaeGoal.NET;
    private Optional<Optional<StagedAlgae>> selectedIntakeGoal = Optional.empty();

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

    private ObjectiveData intakeCoralTarget;
    private ObjectiveData intakeAlgaeTarget;
    private ObjectiveData scoreCoralTarget;
    private ObjectiveData scoreAlgaeTarget;
    private Optional<ObjectiveData> target;
    private Optional<ObjectiveType> typeOverride;

    public ObjectiveTracker(ObjectiveSelectorIO io) {
        System.out.println("[Init] Instantiating ObjectiveTracker");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Objective Tracker", inputs);

        if (inputs.coral != -1) {
            var rack = inputs.coral >> 3 & 0b1111;
            var side = inputs.coral >> 2 & 0b1;
            var level = inputs.coral & 0b11;
            selectedBranch = FieldConstants.Reef.getBranch(rack, side, level);
            inputs.coral = -1;
        }
        if (inputs.algae != -1) {
            selectedAlgaeGoal = AlgaeGoal.values()[inputs.algae];
            inputs.algae = -1;
        }
        if (inputs.intake != -1) {
            selectedIntakeGoal = 
                inputs.intake == 0 ?
                Optional.empty() :
                inputs.intake == 1 ?
                Optional.of(Optional.empty()) :
                Optional.of(Optional.of(FieldConstants.Reef.stagedAlgae[inputs.intake - 2]));
            inputs.intake = -1;
        }

        io.setCoral(
            selectedBranch.pipe.rack.ordinal() << 3 |
            selectedBranch.pipe.side.ordinal() << 2 |
            selectedBranch.level.ordinal()
        );
        io.setAlgae(selectedAlgaeGoal.ordinal());
        io.setIntake(
            selectedIntakeGoal.isEmpty() ? 0 :
            selectedIntakeGoal.get().isEmpty() ? 1 :
            selectedIntakeGoal.get().get().getIndex() + 2
        );

        Logger.recordOutput("Objective Tracker/Selected Branch", selectedBranch.pose.getOurs());
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
        intakeCoralTarget = ObjectiveData.fromParts(closestStationPose, CoralStation.intakePosition, currentPose.getRotation(), ObjectiveType.IntakeCoral);

        var stagedAlgae = new StagedAlgae[] {
            Rack.Rack0.stagedAlgae,
            Rack.Rack1.stagedAlgae,
            Rack.Rack2.stagedAlgae,
            Rack.Rack3.stagedAlgae,
            Rack.Rack4.stagedAlgae,
            Rack.Rack5.stagedAlgae,
        };
        var closestAlgae = Arrays.stream(stagedAlgae).sorted((a,b) -> {
            var aDistance = a.rack.algaeIntakeRobotPose.getOurs().getClosest(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
            var bDistance = b.rack.algaeIntakeRobotPose.getOurs().getClosest(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
            return (int) Math.signum(aDistance - bDistance);
        }).findFirst().get();
        intakeAlgaeTarget = ObjectiveData.fromTotalState(closestAlgae.totalState.getOurs(), currentPose.getRotation(), ObjectiveType.IntakeAlgae);

        this.scoreCoralTarget = ObjectiveData.fromTotalState(selectedBranch.totalState.getOurs(), currentPose.getRotation(), ObjectiveType.ScoreCoral);

        switch (selectedAlgaeGoal) {
            default:
            case PROCESSOR:
                var algaeScoreTarget = Processor.processorTargetPose.getOurs();
                scoreAlgaeTarget = ObjectiveData.fromParts(algaeScoreTarget, Processor.superstructureState, currentPose.getRotation(), ObjectiveType.ScoreAlgae);
            break;
            case OPPONENT_PROCESSOR:
                var algaeScoreTarget2 = Processor.processorTargetPose.getTheirs();
                scoreAlgaeTarget = ObjectiveData.fromParts(algaeScoreTarget2, Processor.superstructureState, currentPose.getRotation(), ObjectiveType.ScoreAlgae);
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
                scoreAlgaeTarget = ObjectiveData.fromParts(closestBargePose, Barge.superstructureState, currentPose.getRotation(), ObjectiveType.ScoreAlgae);
            break;
        }

        Logger.recordOutput("Objective Tracker/Intake/Coral/Target Direction", intakeCoralTarget.targetDirection);
        Logger.recordOutput("Objective Tracker/Intake/Coral/Target Pose", intakeCoralTarget.targetPose);
        Logger.recordOutput("Objective Tracker/Intake/Coral/Target Mechs", intakeCoralTarget.targetState.getMechTransforms());
        Logger.recordOutput("Objective Tracker/Intake/Algae/Target Direction", intakeAlgaeTarget.targetDirection);
        Logger.recordOutput("Objective Tracker/Intake/Algae/Target Pose", intakeAlgaeTarget.targetPose);
        Logger.recordOutput("Objective Tracker/Intake/Algae/Target Mechs", intakeAlgaeTarget.targetState.getMechTransforms());
        Logger.recordOutput("Objective Tracker/Score/Coral/Target Direction", scoreCoralTarget.targetDirection);
        Logger.recordOutput("Objective Tracker/Score/Coral/Target Pose", scoreCoralTarget.targetPose);
        Logger.recordOutput("Objective Tracker/Score/Coral/Target Mechs", scoreCoralTarget.targetState.getMechTransforms());
        Logger.recordOutput("Objective Tracker/Score/Algae/Target Direction", scoreAlgaeTarget.targetDirection);
        Logger.recordOutput("Objective Tracker/Score/Algae/Target Pose", scoreAlgaeTarget.targetPose);
        Logger.recordOutput("Objective Tracker/Score/Algae/Target Mechs", scoreAlgaeTarget.targetState.getMechTransforms());

        cachedBranch = selectedBranch;

        if (typeOverride.isEmpty()) {
            if (hasCoral && hasAlgae) {
                var distToCoral = scoreCoralTarget.targetPose.getTranslation().getDistance(currentPose.getTranslation());
                var distToAlgae = scoreAlgaeTarget.targetPose.getTranslation().getDistance(currentPose.getTranslation());
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

    public ObjectiveData getIntakeCoralObjective() {
        return intakeCoralTarget;
    }
    public ObjectiveData getIntakeAlgaeObjective() {
        return intakeAlgaeTarget;
    }
    public ObjectiveData getScoreCoralObjective() {
        return scoreCoralTarget;
    }
    public ObjectiveData getScoreAlgaeObjective() {
        return scoreAlgaeTarget;
    }
    public Optional<ObjectiveData> getCurrentObjective() {
        return target;
    }

    public void setTypeOverride(Optional<ObjectiveType> typeOverride) {
        this.typeOverride = typeOverride;
    }

    public void moveSelectedBranch(int x, int y) {
        var horiz = Math.floorMod(((selectedBranch.pipe.rack.ordinal() * Side.values().length) + selectedBranch.pipe.side.ordinal() + x), (Rack.values().length * Side.values().length));
        var height = Math.floorMod((selectedBranch.level.ordinal() + y), Level.values().length);
        selectedBranch = FieldConstants.Reef.getBranch(horiz / Side.values().length, Math.floorMod(horiz, Side.values().length), height);
    }

    public void toggleSelectedBranch() {
        if (!placedCoral.remove(selectedBranch)) {
            placedCoral.add(selectedBranch);
        }
    }

    private Branch cachedBranch;
    public Branch getSelectedBranch() {
        return cachedBranch;
    }

    public boolean intakeFromCoralStation() {
        return selectedIntakeGoal.isEmpty();
    }

    public Optional<Optional<StagedAlgae>> getSelectedStagedAlgae() {
        return selectedIntakeGoal;
    }

    public AlgaeGoal getAlgaeGoal() {
        return selectedAlgaeGoal;
    }

    public static class ObjectiveData {
        public final Pose2d targetPose;
        public final SuperstructureState targetState;
        public final Direction targetDirection;
        public final ObjectiveType objectiveType;

        private ObjectiveData(Pose2d targetPose, SuperstructureState targetState, Direction targetDirection, ObjectiveType objectiveType) {
            this.targetPose = targetPose;
            this.targetState = targetState;
            this.targetDirection = targetDirection;
            this.objectiveType = objectiveType;
        }

        public static ObjectiveData fromTotalState(RobotFlippedTotalState totalState, Rotation2d currentRotation, ObjectiveType objectiveType) {
            var direction = totalState.getClosestDirection(currentRotation);
            return new ObjectiveData(
                totalState.getRobotPose(direction),
                totalState.getSuperstructureState(direction),
                direction,
                objectiveType
            );
        }
        public static ObjectiveData fromParts(RobotFlippedRobotPose pose, RobotFlippedSuperstructureState state, Rotation2d currentRotation, ObjectiveType objectiveType) {
            var direction = pose.getClosestDirection(currentRotation);
            return new ObjectiveData(
                pose.get(direction),
                state.get(direction),
                direction,
                objectiveType
            );
        }
        public static ObjectiveData fromRaw(Pose2d pose, SuperstructureState state, Direction direction, ObjectiveType objectiveType) {
            return new ObjectiveData(
                pose,
                state,
                direction,
                objectiveType
            );
        }

        public static class IntakeAlgaeObjective extends ObjectiveData {
            public final StagedAlgae algae;

            private IntakeAlgaeObjective(StagedAlgae algae, Rotation2d currentRotation) {
                var direction = algae.totalState.getOurs().getClosestDirection(currentRotation);
                super(algae.totalState.getOurs().getRobotPose(direction), algae.totalState.getOurs().getSuperstructureState(direction), direction, ObjectiveType.IntakeAlgae);
            }
        }
    }
}