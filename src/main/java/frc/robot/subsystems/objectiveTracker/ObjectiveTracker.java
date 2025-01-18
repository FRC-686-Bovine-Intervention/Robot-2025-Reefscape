package frc.robot.subsystems.objectiveTracker;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Coral;
import frc.robot.constants.FieldConstants.Reef.Level;
import frc.robot.constants.FieldConstants.Reef.Node;
import frc.robot.constants.FieldConstants.Reef.Rack;
import frc.robot.constants.FieldConstants.Reef.Side;
import frc.util.VirtualSubsystem;
import frc.util.rust.iter.Iterator;

public class ObjectiveTracker extends VirtualSubsystem {
    private final ArrayList<Node> placedCoral = new ArrayList<>(36);
    private Node selectedNode = FieldConstants.Reef.nodes[0];

    @Override
    public void periodic() {
        Logger.recordOutput("Selected Coral", selectedNode.pose.getOurs());
        Logger.recordOutput("Placed Coral", Iterator.of(placedCoral).map((node) -> node.pose.getOurs().transformBy(Coral.rackPlacement)).collect_array(Pose3d[]::new));
    }

    public void moveSelectedNode(int x, int y) {
        var horiz = Math.floorMod(((selectedNode.rack.ordinal() * Side.values().length) + selectedNode.side.ordinal() + x), (Rack.values().length * Side.values().length));
        var height = Math.floorMod((selectedNode.level.ordinal() + y), Level.values().length);
        selectedNode = FieldConstants.Reef.getNode(Rack.values()[horiz / Side.values().length], Level.values()[height], Side.values()[Math.floorMod(horiz, Side.values().length)]);
    }

    public void toggleSelectedNode() {
        if (!placedCoral.remove(selectedNode)) {
            placedCoral.add(selectedNode);
        }
    }

    public Node getSelectedNode() {
        return selectedNode;
    }
}
