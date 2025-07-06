package frc.robot.subsystems.superstructure.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import frc.util.mechanismUtil.GearRatio;

public class PivotConstants {
    public static final Transform3d pivotBase = new Transform3d(
        new Translation3d(
            Inches.of(-10),
            Inches.zero(),
            Inches.of(9)
        ),
        Rotation3d.kZero
    );

    public static final Pose2d pivotRobotSpace = new Pose2d(
        new Translation2d(
            PivotConstants.pivotBase.getMeasureX(),
            PivotConstants.pivotBase.getMeasureZ()
        ),
        Rotation2d.kZero
    );

    public static final GearRatio motorToMechanism = new GearRatio()
        .planetary(9)
        .gear(20)
        .gear(66)
        .axle()
        .gear(10)
        .gear(80)
        .axle()
    ;
    public static final GearRatio sensorToMechanism = new GearRatio()
        
    ;

    public static final Angle minAngle = Degrees.of(20);
    public static final Angle maxAngle = Degrees.of(115);
}
