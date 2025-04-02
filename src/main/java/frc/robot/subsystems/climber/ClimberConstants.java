package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.util.mechanismUtil.GearRatio;

public final class ClimberConstants {
    // TBD
    public static final Distance climberElevatorOffset = Inches.of(16.05);
    public static final Distance climberBackwardOffset = Inches.of(6);
    // TBD

    public static final Angle climberMinimumAngle = Rotations.of(0);

    public static final GearRatio sensorToMechanismRatio = new GearRatio()
        .planetary(1.0/5.0)
        .planetary(1.0/5.0)
        .gear(40).gear(50).axle()
    ;

    public static final boolean climberSensorInverted = true;
}
