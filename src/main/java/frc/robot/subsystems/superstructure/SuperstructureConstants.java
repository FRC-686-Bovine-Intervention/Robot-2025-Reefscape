package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants.Algae;
import frc.robot.constants.FieldConstants.Coral;

public class SuperstructureConstants {
    public static final Transform2d coralScoringForwardTransform = new Transform2d(
        new Translation2d(
            Coral.length.plus(Inches.of(1)),
            Inches.zero()
        ),
        new Rotation2d(Degrees.of(35).unaryMinus())
    ).inverse();   

    public static final Transform2d algaeStagedForwardTransform = new Transform2d(
        new Translation2d(
            Algae.radius.plus(Inches.of(1)),
            Inches.zero()
        ),
        Rotation2d.kZero
    ).inverse();

    public static final Transform2d coralIntakeForwardTransform = new Transform2d(
        new Translation2d(
            Inches.of(12),
            Inches.zero()
        ),
        Rotation2d.kZero
    ).inverse();

    // TODO
    public static final Transform2d algaeOuttakeForwardTransform = new Transform2d();
}
