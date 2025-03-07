package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class IntakeConstants {
    public static final Transform3d coralPose = new Transform3d(
        new Translation3d(

        ),
        new Rotation3d()
    );
    public static final Transform3d algaePose = new Transform3d(
        new Translation3d(

        ),
        new Rotation3d()
    );

    public static final boolean coralSensorInverted = true;
    public static final boolean algaeSensorInverted = true;
}
