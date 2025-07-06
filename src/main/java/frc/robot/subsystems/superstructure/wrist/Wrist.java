package frc.robot.subsystems.superstructure.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.util.robotStructure.angle.ArmMech;

public class Wrist {
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    public final ArmMech mech = new ArmMech(WristConstants.wristBase);

    public Wrist(WristIO io) {
        System.out.println("[Init Wrist] Instantiating Wrist with " + io.getClass().getSimpleName());
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Superstructure/Wrist", inputs);

        var angle = getAngle();

        mech.set(angle);

        Logger.recordOutput("Superstructure/Wrist/Angle", angle);
    }

    public Angle getAngle() {
        return WristConstants.sensorToMechanism.applyUnsigned(inputs.encoder.position);
    }
    public AngularVelocity getVelocity() {
        return WristConstants.sensorToMechanism.applyUnsigned(inputs.encoder.velocity);
    }
    public Voltage getVoltage() {
        return inputs.motor.motor.appliedVoltage;
    }
    
    public void setVoltage(Measure<VoltageUnit> voltage) {
        io.setVoltage(voltage);
    }
    public void setAngle(Measure<AngleUnit> angle) {
        io.setAngle(angle);
    }
    public void setFeedForward(Measure<VoltageUnit> feedForward) {
        io.setFeedForward(feedForward);
    }
}
