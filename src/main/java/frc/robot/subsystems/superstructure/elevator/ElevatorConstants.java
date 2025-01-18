package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {
    
    public static final Distance sprocketRadius = Inches.of(1.5);

    public static final Distance pivotOffset = Meters.of(0.050800);

    public static final Distance minimumHeight = Inches.of(26.500000);
    public static final Distance maximumHeight = Inches.of(76.930235);
    public static final Distance maximumLength = maximumHeight.minus(maximumHeight);
}
