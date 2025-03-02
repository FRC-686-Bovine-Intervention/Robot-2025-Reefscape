package frc.robot.subsystems.vision.questnav;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.RobotController;

public class QuestNavIOQuest3S implements QuestNavIO {
    private final NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
    private final NetworkTable nt4Table = nt4Instance.getTable("questnav");

    private final IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
    private final IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();
    
    private final DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
    private final FloatArraySubscriber questPosition = nt4Table.getFloatArrayTopic("position").subscribe(new float[]{0.0f, 0.0f, 0.0f});
    private final FloatArraySubscriber questQuaternion = nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[]{0.0f, 0.0f, 0.0f, 0.0f});
    private final DoubleSubscriber questBatteryPercent = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

    private final MutAngle yawOffset = Radians.mutable(0);

    public QuestNavIOQuest3S() {}

    @Override
    public void updateInputs(QuestNavIOInputs inputs) {
        inputs.isConnected = connected();
        inputs.timestamp = timestamp();
        inputs.batteryPercent = getBatteryPercent();
        inputs.cameraPose = getPose();
    }

    // Zero the relative robot heading.
    public void zeroHeading() {
        yawOffset.mut_replace(new Rotation3d(getQuaternion()).getMeasureZ());
    }

    // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo).
    public void zeroPosition() {
        if (questMiso.get() != 99) {
            questMosi.set(1);
        }
    }

    // Clean up questnav subroutine messages after processing on the headset.
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

    // Gets the Quaternion of the Quest.
    private Quaternion getQuaternion() {
        float[] qqFloats = questQuaternion.get();
        return new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]);
    }

    // Returns the rotation as a Rotation3d object.
    private Rotation3d getRotation() {
        return new Rotation3d(getQuaternion());
    }

    // Returns the position as a Translation3d object.
    private Translation3d getTranslation() {
        float[] questnavPosition = questPosition.get();
        return new Translation3d(questnavPosition[2], -questnavPosition[0], questnavPosition[1]);
    }

    // Gets the estimated pose of the Quest system, factoring in offsets.
    private Pose3d getPose() {
        var translation = getTranslation();
        var rotation = getRotation()
            .minus(new Rotation3d(VecBuilder.fill(0, 0, 1), yawOffset));
        return new Pose3d(translation, rotation);
    }
}
