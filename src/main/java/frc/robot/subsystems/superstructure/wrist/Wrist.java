package frc.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import frc.util.robotStructure.PointOfMass;
import frc.util.robotStructure.angle.ArmMech;

public class Wrist {
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    public final ArmMech mech = new ArmMech(WristConstants.wristBase);
    public final PointOfMass wristMass = new PointOfMass(
        new Translation3d(
            Inches.of(10),
            Inches.zero(),
            Inches.zero()
        ),
        Pounds.of(8.5)
    );

    public Wrist(WristIO io) {
        System.out.println("[Init Wrist] Instantiating Wrist with " + io.getClass().getSimpleName());
        this.io = io;
        mech.addChild(wristMass);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure/Wrist", inputs);

        mech.set(inputs.encoder.position);

        Logger.recordOutput("Superstructure/Pivot/Angle", getAngle());
    }

    public Angle getAngle() {
        return inputs.encoder.position;
    }
    
    public void setVoltage(Measure<VoltageUnit> voltage) {
        io.setVoltage(voltage);
    }

    public void setAngle(Measure<AngleUnit> angle) {
        io.setAngle(angle);
    }
}
