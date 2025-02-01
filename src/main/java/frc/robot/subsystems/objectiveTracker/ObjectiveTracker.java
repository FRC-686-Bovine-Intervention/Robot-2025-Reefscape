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

    enum AlgaeGoal {
        NET,
        PROCESSOR,
        OPPONENT_PROCESSOR
    }

    private final ArrayList<Branch> placedCoral = new ArrayList<>(36);
    private Branch selectedCoral = FieldConstants.Reef.branches[0];
    private AlgaeGoal selectedAlgaeGoal = AlgaeGoal.NET;
    private Optional<Optional<StagedAlgae>> selectedIntakeGoal = Optional.empty();

    public ObjectiveTracker(ObjectiveSelectorIO io) {
        System.out.println("[Init] Creating ObjectiveTracker");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ObjectiveTracker", inputs);

        if (inputs.coral != -1) {
            selectedCoral = FieldConstants.Reef.branches[inputs.coral];
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

        io.setCoral(selectedCoral.getIndex());
        io.setAlgae(selectedAlgaeGoal.ordinal());
        io.setIntake(
            selectedIntakeGoal.isEmpty() ? 0 :
            selectedIntakeGoal.get().isEmpty() ? 1 :
            selectedIntakeGoal.get().get().getIndex() + 2
        );
        
        Logger.recordOutput("ObjectiveTracker/Selected Branch", selectedCoral.branchPose.getOurs());
    }

    public void moveSelectedCoral(int x, int y) {
        var horiz = Math.floorMod(((selectedCoral.rack.ordinal() * Side.values().length) + selectedCoral.side.ordinal() + x), (Rack.values().length * Side.values().length));
        var height = Math.floorMod((selectedCoral.level.ordinal() + y), Level.values().length);
        selectedCoral = FieldConstants.Reef.getBranch(Rack.values()[horiz / Side.values().length], Level.values()[height], Side.values()[Math.floorMod(horiz, Side.values().length)]);
    }

    public void toggleSelectedNode() {
        if (!placedCoral.remove(selectedCoral)) {
            placedCoral.add(selectedCoral);
        }
    }

    public Branch getSelectedNode() {
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