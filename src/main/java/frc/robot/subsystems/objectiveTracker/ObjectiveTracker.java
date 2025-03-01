package frc.robot.subsystems.objectiveTracker;

import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef.Branch;
import frc.robot.constants.FieldConstants.Reef.Level;
import frc.robot.constants.FieldConstants.Reef.Rack;
import frc.robot.constants.FieldConstants.Reef.Side;
import frc.robot.constants.FieldConstants.Reef.StagedAlgae;
import frc.util.VirtualSubsystem;

public class ObjectiveTracker extends VirtualSubsystem {
    private final ObjectiveSelectorIO io;
    private final ObjectiveSelectorIOInputsAutoLogged inputs =
        new ObjectiveSelectorIOInputsAutoLogged();

    public enum AlgaeGoal {
        NET,
        PROCESSOR,
        OPPONENT_PROCESSOR,
        ;
    }

    private final ArrayList<Branch> placedCoral = new ArrayList<>(36);
    private Branch selectedCoral = FieldConstants.Reef.branches[0];
    private AlgaeGoal selectedAlgaeGoal = AlgaeGoal.NET;
    private Optional<Optional<StagedAlgae>> selectedIntakeGoal = Optional.empty();

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
            selectedCoral = FieldConstants.Reef.getBranch(rack, side, level);
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
            selectedCoral.pipe.rack.ordinal() << 3 |
            selectedCoral.pipe.side.ordinal() << 2 |
            selectedCoral.level.ordinal()
        );
        io.setAlgae(selectedAlgaeGoal.ordinal());
        io.setIntake(
            selectedIntakeGoal.isEmpty() ? 0 :
            selectedIntakeGoal.get().isEmpty() ? 1 :
            selectedIntakeGoal.get().get().getIndex() + 2
        );
        
        Logger.recordOutput("Objective Tracker/Selected Branch", selectedCoral.pose.getOurs());
        var setpointState = selectedCoral.level.superstructureStates.getForward();
        Logger.recordOutput("Objective Tracker/Branch Robot Vis/Setpoint/Pivot Angle", setpointState.pivotAngle);
        Logger.recordOutput("Objective Tracker/Branch Robot Vis/Setpoint/Elevator Length", setpointState.elevatorLength);
        Logger.recordOutput("Objective Tracker/Branch Robot Vis/Setpoint/Wrist Angle", setpointState.wristAngle);
        Logger.recordOutput("Objective Tracker/Branch Robot Vis/Robot", selectedCoral.pipe.robotPose.getOurs().getForward());
        Logger.recordOutput("Objective Tracker/Branch Robot Vis/Mechs", setpointState.getMechTransforms());
    }

    public void moveSelectedCoral(int x, int y) {
        var horiz = Math.floorMod(((selectedCoral.pipe.rack.ordinal() * Side.values().length) + selectedCoral.pipe.side.ordinal() + x), (Rack.values().length * Side.values().length));
        var height = Math.floorMod((selectedCoral.level.ordinal() + y), Level.values().length);
        selectedCoral = FieldConstants.Reef.getBranch(horiz / Side.values().length, Math.floorMod(horiz, Side.values().length), height);
    }

    public void toggleSelectedNode() {
        if (!placedCoral.remove(selectedCoral)) {
            placedCoral.add(selectedCoral);
        }
    }

    public Branch getSelectedBranch() {
        return selectedCoral;
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