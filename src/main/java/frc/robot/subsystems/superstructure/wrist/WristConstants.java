package frc.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import frc.util.mechanismUtil.GearRatio;

public class WristConstants {
    public static final Transform3d wristBase = new Transform3d(
        new Translation3d(
            Inches.of(25),
            Inches.zero(),
            Inches.zero()
        ),
        Rotation3d.kZero
    );

    public static final Angle minAngle = Degrees.of(-117.66357421875);
    public static final Angle maxAngle = Degrees.of(90);
    
    public static final GearRatio motorToSensor = new GearRatio()
        .planetary(5)
        .planetary(4)
    ;
    public static final GearRatio sensorToMechanism = new GearRatio()
        .sprocket(20)
        .sprocket(32)
    ;
    public static final GearRatio motorToMechanism = motorToSensor.then(sensorToMechanism);
}
