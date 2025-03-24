package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.FieldConstants.Algae;
import frc.robot.constants.FieldConstants.Coral;

public final class IntakeConstants {
    public static final Transform3d coralPose = new Transform3d(
        new Translation3d(
            Inches.of(5.5).plus(Coral.length.div(2)),
            Inches.zero(),
            Inches.zero()
        ),
        Rotation3d.kZero
    );
    public static final Transform3d algaePose = new Transform3d(
        new Translation3d(
            Inches.of(9.5).plus(Algae.radius),
            Inches.zero(),
            Inches.zero()
        ),
        Rotation3d.kZero
    );

    public static final boolean coralSensorInverted = true;
    public static final boolean algaeSensorInverted = true;
}
