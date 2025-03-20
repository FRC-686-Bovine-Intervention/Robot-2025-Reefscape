package frc.robot.subsystems.vision.questnav;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface QuestNavIO {
    @AutoLog
    public class QuestNavIOInputs {
        public boolean isConnected;
        public double timestamp;
        public double batteryPercent;
        public Pose2d pose = new Pose2d();
    }

    public default void updateInputs(QuestNavIOInputs inputs) {}

    public default void zeroPosition() {}
    public default void cleanUp() {}
}