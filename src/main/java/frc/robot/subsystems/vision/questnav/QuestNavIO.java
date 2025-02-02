package frc.robot.subsystems.vision.questnav;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface QuestNavIO {
    @AutoLog
    public class QuestNavIOInputs {
        public boolean isConnected;
        public double timestamp;
        public double batteryPercent;
        public Pose3d cameraPose;
    }

    public default void updateInputs(QuestNavIOInputs inputs) {}

    public default void zeroHeading() {}
    public default void zeroPosition() {}
    public default void cleanUp() {}
}