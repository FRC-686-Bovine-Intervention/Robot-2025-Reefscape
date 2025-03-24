package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.superstructure.SuperstructureConstants;

public final class IntakeConstants {
    public static final Transform3d coralPose = new Transform3d(
        new Translation3d(
            SuperstructureConstants.wristAxisToCoralTip.getTranslation().getMeasureX(),
            Inches.zero(),
            SuperstructureConstants.wristAxisToCoralTip.getTranslation().getMeasureY()
        ),
        new Rotation3d(
            Degrees.zero(),
            SuperstructureConstants.wristAxisToCoralTip.getRotation().getMeasure().unaryMinus(),
            Degrees.zero()
        )
    );
    public static final Transform3d algaePose = new Transform3d(
        new Translation3d(
            SuperstructureConstants.wristAxisToAlgaeCenter.getTranslation().getMeasureX(),
            Inches.zero(),
            SuperstructureConstants.wristAxisToAlgaeCenter.getTranslation().getMeasureY()
        ),
        new Rotation3d(
            Degrees.zero(),
            SuperstructureConstants.wristAxisToAlgaeCenter.getRotation().getMeasure().unaryMinus(),
            Degrees.zero()
        )
    );

    public static final boolean coralSensorInverted = true;
    public static final boolean algaeSensorInverted = true;
}
