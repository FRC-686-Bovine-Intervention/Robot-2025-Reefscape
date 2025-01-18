package frc.robot.subsystems.superstructure.pivot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

public class PivotConstants {
    public static final Distance pivotX = Meters.of(-0.228600);
    public static final Distance pivotZ = Meters.of(0.254000);

    public static final Transform3d pivotBase = new Transform3d(
        new Translation3d(
            pivotX,
            Meters.of(0),
            pivotZ
        ),
        Rotation3d.kZero
    );
}
