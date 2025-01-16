package frc.robot.subsystems.objectiveTracker;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef.Node;

public class NodeSelectorIOServer implements NodeSelectorIO {
    private final IntegerArrayPublisher nodePublisher;
    private final IntegerArraySubscriber nodeSubscriber;

    public NodeSelectorIOServer() {
        System.out.println("[Init] Creating NodeSelectorIOServer");

        var table = NetworkTableInstance.getDefault().getTable("node_selector");
        nodePublisher = table.getIntegerArrayTopic("node_robot_to_dashboard").publish();
        nodeSubscriber =
            table
                .getIntegerArrayTopic("node_dashboard_to_robot")
                .subscribe(new long[] { -1, -1, -1 }); // rack, level, side
    
        WebServer.start(5801, Filesystem.getDeployDirectory().getPath() + "/node_selector");
    }

    @Override
    public void updateInputs(NodeSelectorIOInputs inputs) {
        for (var value : nodeSubscriber.readQueueValues()) {
            var rackIdx = (int) value[0];
            var levelIdx = (int) value[1];
            var sideIdx = (int) value[2];
            inputs.node = FieldConstants.Reef.getNode(rackIdx, levelIdx, sideIdx);
        }
    }

    @Override
    public void setSelectedNode(Node selected) {
        nodePublisher.set(new long[] {selected.rack.ordinal(), selected.level.ordinal(), selected.side.ordinal()});
    }
}
