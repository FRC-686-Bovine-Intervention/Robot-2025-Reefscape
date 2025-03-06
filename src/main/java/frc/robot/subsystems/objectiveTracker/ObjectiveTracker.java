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
import frc.robot.subsystems.superstructure.Superstructure.RobotFlippedRobotPose;
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

    private Pose2d targetPose = Pose2d.kZero;

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
            selectedIntakeGoal = (inputs.intake > 0) ? (
                Optional.of(Optional.of(FieldConstants.Reef.stagedAlgae[inputs.intake - 1]))
            ) : (
                Optional.empty()
            );
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
        var setpointState = selectedBranch.level.superstructureStates.getForward();
        Logger.recordOutput("Objective Tracker/Branch Robot Vis/Setpoint/Pivot Angle", setpointState.pivotAngle);
        Logger.recordOutput("Objective Tracker/Branch Robot Vis/Setpoint/Elevator Length", setpointState.elevatorLength);
        Logger.recordOutput("Objective Tracker/Branch Robot Vis/Setpoint/Wrist Angle", setpointState.wristAngle);
        Logger.recordOutput("Objective Tracker/Branch Robot Vis/Robot", selectedBranch.pipe.robotPose.getOurs().getForward());
        Logger.recordOutput("Objective Tracker/Branch Robot Vis/Mechs", setpointState.getMechTransforms());
    }

    public void determineGoal(Pose2d currentPose, boolean hasCoral, boolean hasAlgae) {
        final Pose2d reefTargetPose = selectedBranch.pipe.robotPose.getOurs().getClosest(currentPose.getRotation());
        final SuperstructureState reefTargetState = selectedBranch.level.superstructureStates.getClosest(selectedBranch.pipe.robotPose.getOurs().getForward().getRotation(), currentPose.getRotation());
        final Pose2d algaeTargetPose;
        final SuperstructureState algaeTargetState;
        switch (selectedAlgaeGoal) {
            default:
            case PROCESSOR:
                algaeTargetPose = Processor.processorTargetPose.getOurs();
                algaeTargetState = Processor.superstructureState;
            break;
            case OPPONENT_PROCESSOR:
                algaeTargetPose = Processor.processorTargetPose.getTheirs();
                algaeTargetState = Processor.superstructureState;
            break;
            case NET:
                algaeTargetPose = Barge.centerBargePose.getOurs();
                algaeTargetState = Barge.superstructurePosition.getClosest(FieldConstants.netForwardRotation.getOurs(), currentPose.getRotation());
            break;
        }
        final Pose2d intakeTargetPose;
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
            intakeTargetPose = closestStationPose.getClosest(currentPose.getRotation());
            intakeTargetState = CoralStation.intakePosition.getClosest(closestStationPose.getForward().getRotation(), currentPose.getRotation());
        } else {
            var algaeIntake = selectedIntakeGoal.get();
            if (algaeIntake.isEmpty()) {
                intakeTargetPose = Pose2d.kZero;
                intakeTargetState = SuperstructureState.defense;
            } else {
                var stagedAlgae = algaeIntake.get();
                var pose = stagedAlgae.rack.algaeIntakeRobotPose.getOurs();
                intakeTargetPose = pose.getClosest(currentPose.getRotation());
                intakeTargetState = stagedAlgae.algaeLevel.superstructurePosition.getClosest(pose.getForward().getRotation(), currentPose.getRotation());
            }
        }

        Logger.recordOutput("Objective Tracker/Reef/Target Pose", reefTargetPose);
        Logger.recordOutput("Objective Tracker/Reef/Target Mechs", reefTargetState.getMechTransforms());
        Logger.recordOutput("Objective Tracker/Algae/Target Pose", algaeTargetPose);
        Logger.recordOutput("Objective Tracker/Algae/Target Mechs", algaeTargetState.getMechTransforms());
        Logger.recordOutput("Objective Tracker/Intake/Target Pose", intakeTargetPose);
        Logger.recordOutput("Objective Tracker/Intake/Target Mechs", intakeTargetState.getMechTransforms());

        if (hasCoral) {
            targetPose = reefTargetPose;
        } else if(hasAlgae) {
            targetPose = algaeTargetPose;
        } else {
            targetPose = intakeTargetPose;
        }
    }

    public Pose2d getTargetPose() {
        return targetPose;
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

    public Branch getSelectedBranch() {
        return selectedBranch;
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