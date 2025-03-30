package frc.robot.subsystems.objectiveTracker;

import java.util.Arrays;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Filesystem;

public class ReefTrackerIOServer implements ReefTrackerIO {
    private static final String toRobotTable = "/ReefControls/ToRobot";
    private static final String toDashboardTable = "/ReefControls/ToDashboard";

    private static final String modeTopicName = "Mode";
    private static final String coralGoalTopicName = "CoralGoal";
    private static final String algaeGoalTopicName = "AlgaeGoal";
    
    private static final String coralTopicName = "Coral";
    private static final String l1TopicName = "Level1";
    private static final String algaeTopicName = "Algae";
    private static final String coopTopicName = "Coop";
    private static final String priorityListTopicName = "PriorityList";

    private final IntegerSubscriber modeSubscriber;
    private final IntegerSubscriber coralGoalSubscriber;
    private final IntegerSubscriber algaeGoalSubscriber;

    private final IntegerSubscriber coralQueueSubscriber;
    private final IntegerSubscriber l1CountSubscriber; 
    private final IntegerSubscriber algaeQueueSubscriber;
    private final BooleanSubscriber coopSubscriber;
    private final IntegerSubscriber priorityListSubscriber;
    
    private final IntegerPublisher modePublisher;
    private final IntegerPublisher coralGoalPublisher;
    private final IntegerPublisher algaeGoalPublisher;
    
    private final DoublePublisher coralStatePublisher;
    private final IntegerPublisher l1CountPublisher;
    private final IntegerPublisher algaeStatePublisher;
    private final BooleanPublisher coopPublisher;
    private final IntegerPublisher priorityListPublisher;

    public ReefTrackerIOServer() {
        System.out.println("[Init] Creating ReefTrackerIOServer");
    
        WebServer.start(5801, Filesystem.getDeployDirectory().getPath() + "/reef_tracker");

        var inputTable = NetworkTableInstance.getDefault().getTable(toRobotTable);

        modeSubscriber =
            inputTable
                .getIntegerTopic(modeTopicName)
                .subscribe(0, PubSubOption.keepDuplicates(true));
        coralGoalSubscriber = 
            inputTable
                .getIntegerTopic(coralGoalTopicName)
                .subscribe(0, PubSubOption.keepDuplicates(true));
        algaeGoalSubscriber = 
            inputTable
                .getIntegerTopic(algaeGoalTopicName)
                .subscribe(0, PubSubOption.keepDuplicates(true));

        coralQueueSubscriber =
            inputTable
                .getIntegerTopic(coralTopicName)
                .subscribe(0, PubSubOption.keepDuplicates(true));
        l1CountSubscriber =
            inputTable
                .getIntegerTopic(l1TopicName)
                .subscribe(0, PubSubOption.keepDuplicates(true));
        algaeQueueSubscriber =
            inputTable
                .getIntegerTopic(algaeTopicName)
                .subscribe(0, PubSubOption.keepDuplicates(true));
        coopSubscriber =
            inputTable
                .getBooleanTopic(coopTopicName)
                .subscribe(false, PubSubOption.keepDuplicates(true));
        priorityListSubscriber =
            inputTable
                .getIntegerTopic(priorityListTopicName)
                .subscribe(0, PubSubOption.keepDuplicates(true));

        var outputTable = NetworkTableInstance.getDefault().getTable(toDashboardTable);

        modePublisher = outputTable.getIntegerTopic(modeTopicName).publish();
        coralGoalPublisher = outputTable.getIntegerTopic(coralGoalTopicName).publish();
        algaeGoalPublisher = outputTable.getIntegerTopic(algaeGoalTopicName).publish();

        coralStatePublisher = outputTable.getDoubleTopic(coralTopicName).publish();
        l1CountPublisher = outputTable.getIntegerTopic(l1TopicName).publish();
        algaeStatePublisher = outputTable.getIntegerTopic(algaeTopicName).publish();
        coopPublisher = outputTable.getBooleanTopic(coopTopicName).publish();
        priorityListPublisher = outputTable.getIntegerTopic(priorityListTopicName).publish();
    }

    @Override
    public void updateInputs(ReefTrackerIOInputs inputs) {
        if (modeSubscriber.readQueue().length > 0) {
            inputs.mode = (int) modeSubscriber.get();
        }
        if (coralGoalSubscriber.readQueue().length > 0) {
            inputs.coralGoal = (int) coralGoalSubscriber.get();
        }
        if (algaeGoalSubscriber.readQueue().length > 0) {
            inputs.algaeGoal = (int) algaeGoalSubscriber.get();
        }

        var branchQueueValues = coralQueueSubscriber.readQueueValues();
        if (branchQueueValues.length > 0) {
            inputs.branchQueue = Arrays.copyOf(inputs.branchQueue, inputs.branchQueue.length + branchQueueValues.length);
            for (int i = 0; i < branchQueueValues.length; i++) {
                inputs.branchQueue[inputs.branchQueue.length - branchQueueValues.length + i] = (int) branchQueueValues[i];
            }
        }
        if (l1CountSubscriber.readQueue().length > 0) {
            inputs.level1Count = (int) l1CountSubscriber.get();
        }
        var algaeQueueValues = algaeQueueSubscriber.readQueueValues();
        if (algaeQueueValues.length > 0) {
            inputs.algaeQueue = Arrays.copyOf(inputs.algaeQueue, inputs.algaeQueue.length + algaeQueueValues.length);
            for (int i = 0; i < algaeQueueValues.length; i++) {
                inputs.algaeQueue[inputs.algaeQueue.length - algaeQueueValues.length + i] = (int) algaeQueueValues[i];
            }
        }
        if (coopSubscriber.readQueue().length > 0) {
            inputs.coop = coopSubscriber.get();
        }
        if (priorityListSubscriber.readQueue().length > 0) {
            inputs.priorityList = new int[8];
            var n = (int) priorityListSubscriber.get();
            for (int i = inputs.priorityList.length - 1; i >= 0; i--) {
                inputs.priorityList[i] = n & 0b111;
                n >>= 3;
            }
        }
    }

    @Override
    public void setMode(int value) {
        modePublisher.set(value);
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
    public void setCoralState(boolean[] value) {
        long n = 0; // 64-bit
        for (boolean b : value) {
            n = (n << 1) | (b ? 1 : 0);
        }
        coralStatePublisher.set(n);
    }
    @Override
    public void setLevel1Count(int value) {
        l1CountPublisher.set(value);
    }
    @Override
    public void setAlgaeState(boolean[] value) {
        int n = 0; // 32-bit
        for (boolean b : value) {
            n = (n << 1) | (b ? 1 : 0);
        }
        algaeStatePublisher.set(n);
    }
    @Override
    public void setCoopState(boolean value) {
        coopPublisher.set(value);
    }
    @Override
    public void setPriorityList(int[] value) {
        int n = 0; // 32-bit
        for (int b : value) {
            n = (n << 3) | b;
        }
        priorityListPublisher.set(n);
    }
}
