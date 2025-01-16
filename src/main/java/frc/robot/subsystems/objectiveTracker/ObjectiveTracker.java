package frc.robot.subsystems.objectiveTracker;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Coral;
import frc.robot.constants.FieldConstants.Reef.Level;
import frc.robot.constants.FieldConstants.Reef.Node;
import frc.robot.constants.FieldConstants.Reef.Rack;
import frc.robot.constants.FieldConstants.Reef.Side;
import frc.util.VirtualSubsystem;
import frc.util.rust.iter.Iterator;

public class ObjectiveTracker extends VirtualSubsystem {
    private final NodeSelectorIO selectorIO;
    private final NodeSelectorIOInputsAutoLogged selectorInputs =
        new NodeSelectorIOInputsAutoLogged();

    private final ArrayList<Node> placedCoral = new ArrayList<>(36);
    private Node selectedNode = FieldConstants.Reef.nodes[0];

    public ObjectiveTracker(NodeSelectorIO selectorIO) {
        System.out.println("[Init] Creating ObjectiveTracker");
        this.selectorIO = selectorIO;
    }

    @Override
    public void periodic() {
        selectorIO.updateInputs(selectorInputs);
        Logger.processInputs("NodeSelector", selectorInputs);

        if (selectorInputs.node != null) {
            selectedNode = selectorInputs.node;
            selectorInputs.node = null;
        }

        selectorIO.setSelectedNode(selectedNode);
        
        Logger.recordOutput("Selected Coral", selectedNode.pose.getOurs());
        Logger.recordOutput("Placed Coral", Iterator.of(placedCoral).map((node) -> node.pose.getOurs().transformBy(Coral.rackPlacement)).collect_array(Pose3d[]::new));
    }

    public void moveSelectedNode(Direction direction) {
        var horiz = Math.floorMod(((selectedNode.rack.ordinal() * Side.values().length) + selectedNode.side.ordinal() + direction.x), (Rack.values().length * Side.values().length));
        var height = Math.floorMod((selectedNode.level.ordinal() + direction.y), Level.values().length);
        selectedNode = FieldConstants.Reef.getNode(Rack.values()[horiz / Side.values().length], Level.values()[height], Side.values()[Math.floorMod(horiz, Side.values().length)]);
    }

    public void toggleSelectedNode() {
        if (!placedCoral.remove(selectedNode)) {
            placedCoral.add(selectedNode);
        }
    }

    public static enum Direction {
        LEFT(-1, 0),
        RIGHT(1, 0),
        UP(0, 1),
        DOWN(0, -1)
        ;
        public int x;
        public int y;
        Direction(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }
}
