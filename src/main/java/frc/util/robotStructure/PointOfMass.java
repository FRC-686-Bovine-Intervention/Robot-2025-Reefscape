package frc.util.robotStructure;

import static edu.wpi.first.units.Units.Kilograms;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Mass;

public class PointOfMass extends ChildBase {
    public final Mass mass;
    public PointOfMass(Translation3d com, Mass mass) {
        super(new Transform3d(com, Rotation3d.kZero));

        this.mass = mass;
    }
    
    public static Translation3d getCenterOfMass(PointOfMass... masses) {
        var accumTranslation = Translation3d.kZero;
        final var accumMass = Kilograms.mutable(0);
        
        for (var pom : masses) {
            accumMass.mut_acc(pom.mass);
            var massRatio = pom.mass.div(accumMass).baseUnitMagnitude();
            accumTranslation = accumTranslation.interpolate(pom.getRobotRelative().getTranslation(), massRatio);
        }

        return accumTranslation;
    }
}
