package frc.robot.subsystems.vision.questnav;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Joystick;

public class QuestNavIOSim implements QuestNavIO {
    Joystick rotJoystick = new Joystick(1);

    @Override
    public void updateInputs(QuestNavIOInputs inputs) {
        var rotation = Rotation2d.fromRotations(rotJoystick.getRawAxis(2));
        var translation = new Translation2d(Inches.of(10.205609),Inches.of(9.533769)).rotateBy(rotation);

        inputs.batteryPercent = 100;
        inputs.isConnected = true;
        inputs.pose = new Pose3d(new Translation3d(translation), new Rotation3d(rotation));
        inputs.timestamp = System.currentTimeMillis(); 
    }
}
