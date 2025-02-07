package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
import frc.util.robotStructure.PointOfMass;
import frc.util.robotStructure.linear.ExtenderMech;

public class Elevator {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public final ExtenderMech stage2Mech = new ExtenderMech(ElevatorConstants.stage2Base);
    public final PointOfMass stage2Mass = new PointOfMass(
        new Translation3d(
            Inches.of(12),
            Inches.zero(),
            Inches.zero()
        ),
        Pounds.of(4.5)
    );
    public final ExtenderMech stage3Mech = new ExtenderMech(ElevatorConstants.stage3Base);
    public final PointOfMass stage3Mass = new PointOfMass(
        new Translation3d(
            Inches.of(12),
            Inches.zero(),
            Inches.zero()
        ),
        Pounds.of(3.5)
    );
    public final ExtenderMech stage4Mech = new ExtenderMech(ElevatorConstants.stage4Base);
    public final PointOfMass stage4Mass = new PointOfMass(
        new Translation3d(
            Inches.of(6),
            Inches.zero(),
            Inches.zero()
        ),
        Pounds.of(6.5)
    );

    public Elevator(ElevatorIO io) {
        System.out.println("[Init Elevator] Instantiating Elevator with " + io.getClass().getSimpleName());
        this.io = io;
        stage2Mech.addChild(stage2Mass);
        stage3Mech.addChild(stage3Mass);
        stage4Mech.addChild(stage4Mass);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure/Elevator", inputs);

        var stageDist = ElevatorConstants.sprocketRadius.times(inputs.encoder.position.in(Radians));

        stage2Mech.set(stageDist);
        stage3Mech.set(stageDist);
        stage4Mech.set(stageDist);

        Logger.recordOutput("Superstructure/Elevator/Length", getLength());
    }

    public Distance getLength() {
        return ElevatorConstants.sprocketRadius.times(inputs.encoder.position.in(Radians)).times(ElevatorConstants.movingStages);
    }

    public void setVoltage(Measure<VoltageUnit> voltage) {
        io.setVoltage(voltage);
    }
    
    public void setLength(Measure<DistanceUnit> length) {
        io.setLength(length);
    }
}
