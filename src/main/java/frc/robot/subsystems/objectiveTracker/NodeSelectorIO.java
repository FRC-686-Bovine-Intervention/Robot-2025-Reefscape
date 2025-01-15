package frc.robot.subsystems.objectiveTracker;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.constants.FieldConstants.Reef.Node;

public interface NodeSelectorIO {
    @AutoLog
    public static class NodeSelectorIOInputs {
        public Node node;
    }

    public default void updateInputs(NodeSelectorIOInputs inputs) {}
    public default void setSelectedNode(Node node) {}
}
