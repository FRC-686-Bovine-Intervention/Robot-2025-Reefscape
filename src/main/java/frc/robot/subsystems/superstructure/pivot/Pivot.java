package frc.robot.subsystems.superstructure.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import frc.util.robotStructure.angle.ArmMech;

public class Pivot {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    public final ArmMech mech = new ArmMech(PivotConstants.pivotBase);

    public Pivot(PivotIO io) {
        System.out.println("[Init Pivot] Instantiating Pivot with " + io.getClass().getSimpleName());
        this.io = io;
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
