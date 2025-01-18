package frc.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.util.mechanismUtil.GearRatio;

public class WristConstants {
    public static final Transform3d wristBase = new Transform3d(
        new Translation3d(
            Meters.of(0.635000),
            Meters.of(0),
            Meters.of(0)
        ),
        new Rotation3d(
            Degrees.of(0),
            Degrees.of(0),
            Degrees.of(0)
        )
    );
    
    public static final GearRatio motorToMechanism = new GearRatio()
    
    ;
}
