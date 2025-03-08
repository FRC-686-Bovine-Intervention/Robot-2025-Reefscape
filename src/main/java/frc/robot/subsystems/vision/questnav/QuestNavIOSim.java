package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.math.geometry.Pose2d;

public class QuestNavIOSim implements QuestNavIO {
    @Override
    public void updateInputs(QuestNavIOInputs inputs) {
        inputs.batteryPercent = 100;
        inputs.isConnected = true;
        inputs.pose = new Pose2d();
        inputs.timestamp = System.currentTimeMillis(); 
    }

    @Override
    public void zeroPosition() {}

    @Override
    public void cleanUp() {}
}
