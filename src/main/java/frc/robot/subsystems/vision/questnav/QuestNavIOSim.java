package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.math.geometry.Pose3d;

public class QuestNavIOSim implements QuestNavIO {
    @Override
    public void updateInputs(QuestNavIOInputs inputs) {
        inputs.batteryPercent = 100;
        inputs.isConnected = true;
        inputs.pose = Pose3d.kZero;
        inputs.timestamp = System.currentTimeMillis(); 
    }
}
