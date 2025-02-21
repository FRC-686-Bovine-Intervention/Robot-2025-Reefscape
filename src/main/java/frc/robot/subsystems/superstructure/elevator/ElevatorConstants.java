package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.util.mechanismUtil.GearRatio;

public class ElevatorConstants {
    public static final Distance pivotOffset = Inches.zero();
    public static final Transform3d stage2Base = new Transform3d(
        new Translation3d(
            Inches.of(-0.5),
            Inches.zero(),
            pivotOffset
        ),
        Rotation3d.kZero
    );
    public static final Transform3d stage3Base = new Transform3d(
        new Translation3d(
            Inches.of(0.5),
            Inches.zero(),
            Inches.zero()
        ),
        Rotation3d.kZero
    );
    public static final Transform3d stage4Base = new Transform3d(
        new Translation3d(
            Inches.of(0.5),
            Inches.zero(),
            Inches.zero()
        ),
        Rotation3d.kZero
    );

    public static final int movingStageCount = 3;
    
    public static final Distance sprocketRadius = Inches.of(1.273).div(2);
    // public static final Distance sprocketRadius = Inches.of(0.25).times(16).div(Math.PI*2);

    public static final Distance stageExtension = Inches.of(17);
    public static final Distance maximumLength = stageExtension.times(movingStageCount);
    public static final Distance minimumHeight = Inches.of(26.5);
    public static final Distance maximumHeight = minimumHeight.plus(maximumLength);

    public static final GearRatio motorToMechanism = new GearRatio()
        .planetary(1.0/3.0)
        .planetary(1.0/3.0)
    ;
    public static final GearRatio sensorToMechanism = new GearRatio()
        .gear(90)
        .gear(18)
        .axle()
    ;
}
