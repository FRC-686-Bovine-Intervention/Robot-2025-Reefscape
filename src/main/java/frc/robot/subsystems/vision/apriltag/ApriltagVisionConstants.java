package frc.robot.subsystems.vision.apriltag;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.FieldConstants;
import frc.util.PoseBoundingBoxUtil.BoundingBox;

public class ApriltagVisionConstants {
    public static final Distance zMargin = Inches.of(6);
    public static final BoundingBox acceptableFieldBox = BoundingBox.rectangle(
        new Translation2d(

        ),
        new Translation2d(
            FieldConstants.fieldLength,
            FieldConstants.fieldWidth
        )
    );
}
