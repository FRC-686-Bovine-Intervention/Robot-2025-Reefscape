package frc.robot.subsystems.superstructure.pivot;

import java.util.stream.IntStream;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
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

        var array = new boolean[64];
        for (int i = 0; i < 64; i++) {
            array[i] = (inputs.rightMotorFaults.faults & (1 << i)) != 0;
        }
        Logger.recordOutput("DEBVUAG/bits", array);

        mech.set(inputs.encoder.position);

        Logger.recordOutput("Superstructure/Pivot/Angle", getAngle());
    }

    public Angle getAngle() {
        return inputs.encoder.position;
    }
    public AngularVelocity getVelocity() {
        return inputs.encoder.velocity;
    }
    public Voltage getVoltage() {
        return inputs.leftMotor.motor.appliedVoltage;
    }

    public void setAngle(Measure<AngleUnit> angle) {
        io.setPosition(angle);
    }
    public void setVoltage(Measure<VoltageUnit> voltage) {
        io.setVoltage(voltage);
    }
    public void setFeedForward(Measure<VoltageUnit> feedForward) {
        io.setFeedForward(feedForward);
    }

    public void setCoastMode() {
        io.setCoastMode();
    }
}
