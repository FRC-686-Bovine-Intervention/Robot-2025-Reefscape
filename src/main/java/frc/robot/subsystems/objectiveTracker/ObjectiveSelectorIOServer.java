package frc.robot.subsystems.objectiveTracker;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;

public class ObjectiveSelectorIOServer implements ObjectiveSelectorIO {
    private final IntegerPublisher coralPublisher;
    private final IntegerSubscriber coralSubscriber;
    private final IntegerPublisher algaePublisher;
    private final IntegerSubscriber algaeSubscriber;
    private final IntegerPublisher intakePublisher;
    private final IntegerSubscriber intakeSubscriber;

    public ObjectiveSelectorIOServer() {
        System.out.println("[Init] Creating ObjectiveSelectorIOServer");

        var table = NetworkTableInstance.getDefault().getTable("objective_selector");
        coralPublisher = table.getIntegerTopic("coral_robot_to_dashboard").publish();
        coralSubscriber = table.getIntegerTopic("coral_dashboard_to_robot").subscribe(-1);
        algaePublisher = table.getIntegerTopic("algae_robot_to_dashboard").publish();
        algaeSubscriber = table.getIntegerTopic("algae_dashboard_to_robot").subscribe(-1);
        intakePublisher = table.getIntegerTopic("intake_robot_to_dashboard").publish();
        intakeSubscriber = table.getIntegerTopic("intake_dashboard_to_robot").subscribe(-1);
    
        WebServer.start(686, Filesystem.getDeployDirectory().getPath() + "/objective_selector");
    }

    @Override
    public void updateInputs(ObjectiveSelectorIOInputs inputs) {
        if (coralSubscriber.readQueueValues().length > 0) {
            inputs.coral = (int) coralSubscriber.get();
        }
        if (algaeSubscriber.readQueueValues().length > 0) {
            inputs.algae = (int) algaeSubscriber.get();
        }
        if (intakeSubscriber.readQueueValues().length > 0) {
            inputs.intake = (int) intakeSubscriber.get();
        }
    }

    @Override
    public void setCoral(int objective) {
        coralPublisher.set(objective);
    }

    @Override
    public void setAlgae(int objective) {
        algaePublisher.set(objective);
    }

    @Override
    public void setIntake(int objective) {
        intakePublisher.set(objective);
    }
}
