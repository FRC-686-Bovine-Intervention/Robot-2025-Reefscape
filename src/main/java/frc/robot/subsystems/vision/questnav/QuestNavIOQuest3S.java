package frc.robot.subsystems.vision.questnav;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

public class QuestNavIOQuest3S implements QuestNavIO {
  // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
  NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  NetworkTable nt4Table = nt4Instance.getTable("questnav");
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

  // Subscribe to the Network Tables questnav data topics
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private FloatArraySubscriber questPosition = nt4Table.getFloatArrayTopic("position").subscribe(new float[]{0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questQuaternion = nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[]{0.0f, 0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questEulerAngles = nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[]{0.0f, 0.0f, 0.0f});
  private IntegerSubscriber questFrameCount = nt4Table.getIntegerTopic("frameCount").subscribe(0);
  private DoubleSubscriber questBatteryPercent = nt4Table.getDoubleTopic("device/batteryPercent").subscribe(0.0f);
  private BooleanSubscriber questIsTracking = nt4Table.getBooleanTopic("device/isTracking").subscribe(false);
  private IntegerSubscriber questTrackingLostCount = nt4Table.getIntegerTopic("device/trackingLostCounter").subscribe(0);

  /** Subscriber for heartbeat requests */
  private final DoubleSubscriber heartbeatRequestSub = nt4Table.getDoubleTopic("heartbeat/quest_to_robot").subscribe(0.0);
  /** Publisher for heartbeat responses */
  private final DoublePublisher heartbeatResponsePub = nt4Table.getDoubleTopic("heartbeat/robot_to_quest").publish();
  /** Last processed heartbeat request ID */
  private double lastProcessedHeartbeatId = 0;

    public QuestNavIOQuest3S() {
        zeroPosition();
    }

    @Override
    public void updateInputs(QuestNavIOInputs inputs) {
        inputs.isConnected = connected();
        inputs.timestamp = timestamp();
        inputs.batteryPercent = getBatteryPercent();
        inputs.isTracking = getTrackingStatus();
        inputs.frameCount = getFrameCount();
        inputs.trackingLostCount = getTrackingLostCount();
        inputs.pose = getPose();
    }

    @Override
    public void cleanUp() {
        this.processHeartbeat();
        this.cleanUpQuestNavMessages();
    }

    // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo).
    @Override
    public void zeroPosition() {
        if (questMiso.get() != 99) {
            questMosi.set(1);
        }
    }

    public void processHeartbeat() {
        double requestId = heartbeatRequestSub.get();
        // Only respond to new requests to avoid flooding
        if (requestId > 0 && requestId != lastProcessedHeartbeatId) {
          heartbeatResponsePub.set(requestId);
          lastProcessedHeartbeatId = requestId;
        }
    }

    // Clean up questnav subroutine messages after processing on the headset.
    public void cleanUpQuestNavMessages() {
        if (questMiso.get() == 99) {
            questMosi.set(0);
        }
    }

    // Returns if the Quest is connected.
    private boolean connected() {
        return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
    }

    // Gets the Quests's timestamp in NT Server Time.
    private double timestamp() {
        return questTimestamp.getAtomic().serverTime;
    }

    // Gets the battery percent of the Quest.
    private double getBatteryPercent() {
        return questBatteryPercent.get();
    }

    // Gets the current tracking state of the Quest. 
    private boolean getTrackingStatus() {
        return questIsTracking.get();
    }

    // Gets the current frame count from the Quest headset.
    private long getFrameCount() {
        return questFrameCount.get();
    }

    // Gets the number of tracking lost events since the Quest connected to the robot. 
    private long getTrackingLostCount() {
        return questTrackingLostCount.get();
    }
        
    // Returns the position as a Translation3d object.
    private Translation3d getTranslation() {
        float[] questnavPosition = questPosition.get();
        return new Translation3d(questnavPosition[2], -questnavPosition[0], questnavPosition[1]);
    }

    // Returns the rotation as a Rotation3d object.
    public Rotation3d getRawRotation() {
        float[] eulerAngles = questEulerAngles.get();
        return new Rotation3d(Degrees.of(-eulerAngles[2]), Degrees.of(eulerAngles[0]), Degrees.of(-eulerAngles[1]));
    }

    // Gets the estimated pose of the Quest system, factoring in offsets.
    private Pose3d getPose() {
        return new Pose3d(getTranslation(), getRawRotation());
    }
}
