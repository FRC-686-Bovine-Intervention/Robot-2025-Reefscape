package frc.robot.subsystems.objectiveTracker;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Coral;
import frc.robot.constants.FieldConstants.Reef.Level;
import frc.robot.constants.FieldConstants.Reef.Branch;
import frc.robot.constants.FieldConstants.Reef.Rack;
import frc.robot.constants.FieldConstants.Reef.Side;
import frc.util.VirtualSubsystem;
import frc.util.rust.iter.Iterator;

public class ObjectiveTracker extends VirtualSubsystem {
    private final ObjectiveSelectorIO selectorIO;
    private final ObjectiveSelectorIOInputsAutoLogged selectorInputs =
        new ObjectiveSelectorIOInputsAutoLogged();

    private final ArrayList<Branch> placedCoral = new ArrayList<>(36);
    private Branch selectedCoral = FieldConstants.Reef.nodes[0];
    private int selectedAlgae = 0;
    private int selectedIntake = 0;

    public ObjectiveTracker(ObjectiveSelectorIO selectorIO) {
        System.out.println("[Init] Creating ObjectiveTracker");
        this.selectorIO = selectorIO;
    }

    @Override
    public void periodic() {
        selectorIO.updateInputs(selectorInputs);
        Logger.processInputs("ObjectiveTracker", selectorInputs);

        if (selectorInputs.coral != -1) {
            selectedCoral = FieldConstants.Reef.nodes[selectorInputs.coral];
            selectorInputs.coral = -1;
        }
        if (selectorInputs.algae != -1) {
            selectedAlgae = selectorInputs.algae;
            selectorInputs.algae = -1;
        }
        if (selectorInputs.intake != -1) {
            selectedIntake = selectorInputs.intake;
            selectorInputs.intake = -1;
        }

        selectorIO.setCoral(selectedCoral.getIndex());
        selectorIO.setAlgae(selectedAlgae);
        selectorIO.setIntake(selectedIntake);
        
        Logger.recordOutput("Selected Coral", selectedCoral.branchPose.getOurs());
        Logger.recordOutput("Placed Coral", Iterator.of(placedCoral).map((node) -> node.branchPose.getOurs().transformBy(Coral.rackPlacement)).collect_array(Pose3d[]::new));
    }

    public void moveSelectedCoral(int x, int y) {
        var horiz = Math.floorMod(((selectedCoral.rack.ordinal() * Side.values().length) + selectedCoral.side.ordinal() + x), (Rack.values().length * Side.values().length));
        var height = Math.floorMod((selectedCoral.level.ordinal() + y), Level.values().length);
        selectedCoral = FieldConstants.Reef.getNode(Rack.values()[horiz / Side.values().length], Level.values()[height], Side.values()[Math.floorMod(horiz, Side.values().length)]);
    }

    public void toggleSelectedNode() {
        if (!placedCoral.remove(selectedCoral)) {
            placedCoral.add(selectedCoral);
        }
    }

    public Branch getSelectedNode() {
        return selectedCoral;
    }
}
