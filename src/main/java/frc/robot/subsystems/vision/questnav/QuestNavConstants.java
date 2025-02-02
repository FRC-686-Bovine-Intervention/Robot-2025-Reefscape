package frc.robot.subsystems.vision.questnav;

import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConstants;
import frc.util.robotStructure.CameraMount;

public class QuestNavConstants {
    public static class QuestNavCameraConstants extends CameraConstants {
        public QuestNavCameraConstants(String hardwareName, CameraMount mount) {
            super(hardwareName, mount);
        }
    }

    public static final QuestNavCameraConstants questNavCamera = new QuestNavCameraConstants(
        "Meta Quest 3S",
        VisionConstants.questNavMount
    );
}
