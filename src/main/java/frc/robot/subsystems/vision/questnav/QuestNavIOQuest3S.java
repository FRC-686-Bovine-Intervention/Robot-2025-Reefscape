package frc.robot.subsystems.vision.questnav;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

public class QuestNavIOQuest3S implements QuestNavIO {
    private final NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
    private final NetworkTable nt4Table = nt4Instance.getTable("questnav");

    private final IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
    private final IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();
    
    private final DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
    private final FloatArraySubscriber questPosition = nt4Table.getFloatArrayTopic("position").subscribe(new float[]{0.0f, 0.0f, 0.0f});
    private final FloatArraySubscriber questEulerAngles = nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[]{0.0f, 0.0f, 0.0f});
    private final DoubleSubscriber questBatteryPercent = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

    public QuestNavIOQuest3S() {
        zeroPosition();
    }

    @Override
    public void updateInputs(QuestNavIOInputs inputs) {
        inputs.isConnected = connected();
        inputs.timestamp = timestamp();
        inputs.batteryPercent = getBatteryPercent();
        inputs.pose = getPose();
    }

    // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo).
    @Override
    public void zeroPosition() {
        if (questMiso.get() != 99) {
            questMosi.set(1);
        }
    }

    // Clean up questnav subroutine messages after processing on the headset.
    @Override
    public void cleanUp() {
        if (questMiso.get() == 99) {
            questMosi.set(0);
        }
    }

    // Returns if the Quest is connected.
    private boolean connected() {
        return (RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000 < 250;
    }

    // Gets the Quests's timestamp.
    private double timestamp() {
        return questTimestamp.get();
    }

    // Gets the battery percent of the Quest.
    private double getBatteryPercent() {
        return questBatteryPercent.get();
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
