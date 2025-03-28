package frc.robot.subsystems.objectiveTracker;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Filesystem;

public class ReefTrackerIOServer implements ReefTrackerIO {
    private static final String toRobotTable = "/ReefControls/ToRobot";
    private static final String toDashboardTable = "/ReefControls/ToDashboard";
    private static final String coralGoalTopicName = "CoralGoal";
    private static final String algaeGoalTopicName = "AlgaeGoal";
    private static final String selectedLevelTopicName = "SelectedLevel";
    private static final String l1TopicName = "Level1";
    private static final String l2TopicName = "Level2";
    private static final String l3TopicName = "Level3";
    private static final String l4TopicName = "Level4";
    private static final String algaeTopicName = "Algae";
    private static final String coopTopicName = "Coop";

    private final IntegerSubscriber coralGoalSubscriber;
    private final IntegerSubscriber algaeGoalSubscriber;

    private final IntegerPublisher coralGoalPublisher;
    private final IntegerPublisher algaeGoalPublisher;
    
    private final IntegerSubscriber selectedLevelSubscriber;
    private final IntegerSubscriber l1StateSubscriber;
    private final IntegerSubscriber l2StateSubscriber;
    private final IntegerSubscriber l3StateSubscriber;
    private final IntegerSubscriber l4StateSubscriber;
    private final IntegerSubscriber algaeStateSubscriber;
    private final BooleanSubscriber coopStateSubscriber;

    private final IntegerPublisher selectedLevelPublisher;
    private final IntegerPublisher l1StatePublisher;
    private final IntegerPublisher l2StatePublisher;
    private final IntegerPublisher l3StatePublisher;
    private final IntegerPublisher l4StatePublisher;
    private final IntegerPublisher algaeStatePublisher;
    private final BooleanPublisher coopStatePublisher;

    public ReefTrackerIOServer() {
        System.out.println("[Init] Creating ReefTrackerIOServer");
    
        WebServer.start(5801, Filesystem.getDeployDirectory().getPath() + "/reef_tracker");

        var inputTable = NetworkTableInstance.getDefault().getTable(toRobotTable);

        coralGoalSubscriber = 
            inputTable
                .getIntegerTopic(coralGoalTopicName)
                .subscribe(0, PubSubOption.keepDuplicates(true));
        algaeGoalSubscriber = 
            inputTable
                .getIntegerTopic(algaeGoalTopicName)
                .subscribe(0, PubSubOption.keepDuplicates(true));

        selectedLevelSubscriber =
            inputTable
                .getIntegerTopic(selectedLevelTopicName)
                .subscribe(0, PubSubOption.keepDuplicates(true));
        l1StateSubscriber =
            inputTable.getIntegerTopic(l1TopicName).subscribe(0, PubSubOption.keepDuplicates(true));
        l2StateSubscriber =
            inputTable.getIntegerTopic(l2TopicName).subscribe(0, PubSubOption.keepDuplicates(true));
        l3StateSubscriber =
            inputTable.getIntegerTopic(l3TopicName).subscribe(0, PubSubOption.keepDuplicates(true));
        l4StateSubscriber =
            inputTable.getIntegerTopic(l4TopicName).subscribe(0, PubSubOption.keepDuplicates(true));
        algaeStateSubscriber =
            inputTable.getIntegerTopic(algaeTopicName).subscribe(0, PubSubOption.keepDuplicates(true));
        coopStateSubscriber =
            inputTable
                .getBooleanTopic(coopTopicName)
                .subscribe(false, PubSubOption.keepDuplicates(true));

        var outputTable = NetworkTableInstance.getDefault().getTable(toDashboardTable);

        coralGoalPublisher = outputTable.getIntegerTopic(coralGoalTopicName).publish();
        algaeGoalPublisher = outputTable.getIntegerTopic(algaeGoalTopicName).publish();

        selectedLevelPublisher = outputTable.getIntegerTopic(selectedLevelTopicName).publish();
        l1StatePublisher = outputTable.getIntegerTopic(l1TopicName).publish();
        l2StatePublisher = outputTable.getIntegerTopic(l2TopicName).publish();
        l3StatePublisher = outputTable.getIntegerTopic(l3TopicName).publish();
        l4StatePublisher = outputTable.getIntegerTopic(l4TopicName).publish();
        algaeStatePublisher = outputTable.getIntegerTopic(algaeTopicName).publish();
        coopStatePublisher = outputTable.getBooleanTopic(coopTopicName).publish();
    }

    @Override
    public void updateInputs(ReefTrackerIOInputs inputs) {
        if (coralGoalSubscriber.readQueue().length > 0) {
            inputs.coralGoal = (int) coralGoalSubscriber.get();
        }
        if (algaeGoalSubscriber.readQueue().length > 0) {
            inputs.algaeGoal = (int) algaeGoalSubscriber.get();
        }

        if (selectedLevelSubscriber.readQueue().length > 0) {
            inputs.selectedLevel = (int) selectedLevelSubscriber.get();
        }
        if (l1StateSubscriber.readQueue().length > 0) {
            inputs.level1State = (int) l1StateSubscriber.get();
        }
        if (l2StateSubscriber.readQueue().length > 0) {
            inputs.level2State = (int) l2StateSubscriber.get();
        }
        if (l3StateSubscriber.readQueue().length > 0) {
            inputs.level3State = (int) l3StateSubscriber.get();
        }
        if (l4StateSubscriber.readQueue().length > 0) {
            inputs.level4State = (int) l4StateSubscriber.get();
        }
        if (algaeStateSubscriber.readQueue().length > 0) {
            inputs.algaeState = (int) algaeStateSubscriber.get();
        }
        if (coopStateSubscriber.readQueue().length > 0) {
            inputs.coopState = coopStateSubscriber.get();
        }
    }

    @Override
    public void setCoralGoal(int value) {
        coralGoalPublisher.set(value);
    }

    @Override
    public void setAlgaeGoal(int value) {
        algaeGoalPublisher.set(value);
    }

    @Override
    public void setSelectedLevel(int value) {
      selectedLevelPublisher.set(value);
    }
  
    @Override
    public void setLevel1State(int value) {
      l1StatePublisher.set(value);
    }
  
    @Override
    public void setLevel2State(int value) {
      l2StatePublisher.set(value);
    }
  
    @Override
    public void setLevel3State(int value) {
      l3StatePublisher.set(value);
    }
  
    @Override
    public void setLevel4State(int value) {
      l4StatePublisher.set(value);
    }
  
    @Override
    public void setAlgaeState(int value) {
      algaeStatePublisher.set(value);
    }
  
    @Override
    public void setCoopState(boolean value) {
      coopStatePublisher.set(value);
    }  
}
