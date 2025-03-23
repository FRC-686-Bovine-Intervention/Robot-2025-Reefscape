package frc.robot.subsystems.objectiveTracker;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
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

    private Optional<Pose2d> targetPose = Optional.empty();
    private Direction reefTargetDirection = Direction.Forward;
    private Direction algaeTargetDirection = Direction.Forward;
    private Direction intakeTargetDirection = Direction.Forward;
    private Direction targetDirection = Direction.Forward;

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
        final RobotFlippedTotalState reefTotalState = selectedBranch.totalState.getOurs();
        reefTargetDirection = reefTotalState.getClosestDirection(currentPose.getRotation());
        final Pose2d reefTargetPose = reefTotalState.getRobotPose(reefTargetDirection);
        final SuperstructureState reefTargetState = reefTotalState.getSuperstructureState(reefTargetDirection);

        final RobotFlippedRobotPose algaeTarget;
        final Pose2d algaeTargetPose;
        final SuperstructureState algaeTargetState;
        switch (selectedAlgaeGoal) {
            default:
            case PROCESSOR:
                algaeTarget = Processor.processorTargetPose.getOurs();
                algaeTargetDirection = algaeTarget.getClosestDirection(currentPose.getRotation());
                algaeTargetPose = algaeTarget.get(algaeTargetDirection);
                algaeTargetState = Processor.superstructureState.get(algaeTargetDirection);
            break;
            case OPPONENT_PROCESSOR:
                algaeTarget = Processor.processorTargetPose.getTheirs();
                algaeTargetDirection = algaeTarget.getClosestDirection(currentPose.getRotation());
                algaeTargetPose = algaeTarget.get(algaeTargetDirection);
                algaeTargetState = Processor.superstructureState.get(algaeTargetDirection);
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
                algaeTarget = closestBargePose;
                algaeTargetDirection = algaeTarget.getClosestDirection(currentPose.getRotation());
                algaeTargetPose = algaeTarget.get(algaeTargetDirection);
                algaeTargetState = Barge.superstructureState.get(algaeTargetDirection);
            break;
        }

        final Optional<RobotFlippedRobotPose> intakeTarget;
        final Optional<Pose2d> intakeTargetPose;
        final SuperstructureState intakeTargetState;
        if (selectedIntakeGoal.isEmpty()) {
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
            intakeTarget = Optional.of(closestStationPose);
            intakeTargetDirection = closestStationPose.getClosestDirection(currentPose.getRotation());
            intakeTargetPose = Optional.of(closestStationPose.get(intakeTargetDirection));
            intakeTargetState = CoralStation.intakePosition.get(intakeTargetDirection);
        } else {
            var algaeIntake = selectedIntakeGoal.get();
            if (algaeIntake.isEmpty()) {
                intakeTarget = Optional.empty();
                intakeTargetPose = Optional.empty();
                intakeTargetDirection = Direction.Forward;
                intakeTargetState = SuperstructureState.defense;
            } else {
                var stagedAlgae = algaeIntake.get();
                var target = stagedAlgae.rack.algaeIntakeRobotPose.getOurs();
                intakeTarget = Optional.of(target);
                intakeTargetDirection = target.getClosestDirection(currentPose.getRotation());
                intakeTargetPose = Optional.of(target.get(intakeTargetDirection));
                intakeTargetState = stagedAlgae.algaeLevel.superstructurePosition.get(intakeTargetDirection);
            }
        }

        Logger.recordOutput("Objective Tracker/Reef/Target Direction", reefTargetDirection);
        Logger.recordOutput("Objective Tracker/Reef/Target Pose", reefTargetPose);
        Logger.recordOutput("Objective Tracker/Reef/Target Mechs", reefTargetState.getMechTransforms());
        Logger.recordOutput("Objective Tracker/Algae/Target Direction", algaeTargetDirection);
        Logger.recordOutput("Objective Tracker/Algae/Target Pose", algaeTargetPose);
        Logger.recordOutput("Objective Tracker/Algae/Target Mechs", algaeTargetState.getMechTransforms());
        Logger.recordOutput("Objective Tracker/Intake/Target Direction", intakeTargetDirection);
        Logger.recordOutput("Objective Tracker/Intake/Target Pose", intakeTargetPose.orElse(Pose2d.kZero));
        Logger.recordOutput("Objective Tracker/Intake/Target Mechs", intakeTargetState.getMechTransforms());

        cachedBranch = selectedBranch;

        if (hasCoral) {
            targetPose = Optional.of(reefTargetPose);
            targetDirection = reefTargetDirection;
        } else if(hasAlgae) {
            targetPose = Optional.of(algaeTargetPose);
            targetDirection = algaeTargetDirection;
        } else {
            targetPose = intakeTargetPose;
            targetDirection = intakeTargetDirection;
        }
    }

    public Optional<Pose2d> getTargetPose() {
        return targetPose;
    }
    public Direction getTargetDirection() {
        return targetDirection;
    }
    public Direction getReefTargetDirection() {
        return reefTargetDirection;
    }
    public Direction getAlgaeTargetDirection() {
        return algaeTargetDirection;
    }
    public Direction getIntakeTargetDirection() {
        return intakeTargetDirection;
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
}