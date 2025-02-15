package frc.robot.subsystems.superstructure.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.util.mechanismUtil.GearRatio;
import frc.util.mechanismUtil.Wheel;

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

    public static final GearRatio pivotMotorToMechanism = new GearRatio()
        .planetary(1.0/5.0)
        .planetary(1.0/5.0)
        .gear(20)
        .gear(66)
        .axle()
        .gear(10)
        .gear(120)
        .axle()
    ;
    public static final GearRatio pivotSensorToMechanism = new GearRatio()
        
    ;

    public static final GearRatio climberMotorToMechanism = new GearRatio()
        .planetary(1.0 / 9.0)
        .gear(44)
        .gear(44)
        .axle()
    ; 
    public static final Wheel climberMotorRotationToLinearDistance = Wheel.diameter(Inches.of(1.273));

    public static final Angle minAngle = Degrees.of(20);
    public static final Angle maxAngle = Degrees.of(115);

    public static final Distance pivotToChain = Meters.of(1);
    public static final Distance pivotToClimberMotor = Meters.of(1);
}
