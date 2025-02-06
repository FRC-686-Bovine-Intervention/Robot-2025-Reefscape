package frc.robot.subsystems.superstructure.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import frc.util.robotStructure.angle.ArmMech;

public class Pivot {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    public final ArmMech mech = new ArmMech(PivotConstants.pivotBase);
    public final PointOfMass stage1Mass = new PointOfMass(
        new Translation3d(
            Inches.of(12),
            Inches.zero(),
            Inches.zero()
        ),
        Pounds.of(5)
    );

    public Pivot(PivotIO io) {
        System.out.println("[Init Pivot] Instantiating Pivot with " + io.getClass().getSimpleName());
        this.io = io;
        mech.addChild(stage1Mass);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure/Pivot", inputs);

        mech.set(inputs.encoder.position);

        Logger.recordOutput("Superstructure/Pivot/Angle", getAngle());
    }

    public Angle getAngle() {
        return inputs.encoder.position;
    }

    public void setPivot(Measure<AngleUnit> angle) {
        io.setPosition(angle);
    }
    public void setVoltage(Measure<VoltageUnit> voltage) {
        io.setVoltage(voltage);
    }
}
