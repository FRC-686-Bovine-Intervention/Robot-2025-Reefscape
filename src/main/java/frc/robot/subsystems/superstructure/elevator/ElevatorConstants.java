package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.util.mechanismUtil.GearRatio;

public class ElevatorConstants {
    
    public static final Distance sprocketRadius = Inches.of(1.432).div(2);

    public static final Distance pivotOffset = Meters.of(0.050800);
    public static final Transform3d elevatorBase = new Transform3d(
        new Translation3d(
            Meters.of(-0.088900),
            Meters.of(0),
            pivotOffset
        ),
        new Rotation3d(
            Degrees.of(0),
            Degrees.of(0),
            Degrees.of(0)
        )
    );

    public static final Distance minimumHeight = Inches.of(26.500000);
    public static final Distance maximumHeight = Inches.of(76.930235);
    public static final Distance maximumLength = maximumHeight.minus(maximumHeight);

    public static final GearRatio motorToMechanism = new GearRatio()
        .planetary(1.0/9.0)
    ;
    public static final GearRatio sensorToMechanism = new GearRatio()
        .gear(10)
        .gear(40)
        .axle()
    ;
}
