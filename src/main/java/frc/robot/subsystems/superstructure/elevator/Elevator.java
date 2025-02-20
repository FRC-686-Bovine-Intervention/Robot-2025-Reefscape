package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.util.robotStructure.linear.ExtenderMech;

public class Elevator {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public final ExtenderMech stage2Mech = new ExtenderMech(ElevatorConstants.stage2Base);
    public final ExtenderMech stage3Mech = new ExtenderMech(ElevatorConstants.stage3Base);
    public final ExtenderMech stage4Mech = new ExtenderMech(ElevatorConstants.stage4Base);

    public Elevator(ElevatorIO io) {
        System.out.println("[Init Elevator] Instantiating Elevator with " + io.getClass().getSimpleName());
        this.io = io;
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
    public LinearVelocity getVelocity() {
        return ElevatorConstants.sprocketRadius.times(inputs.encoder.velocity.in(RadiansPerSecond)).per(Second).times(ElevatorConstants.movingStages);
    }
    public Voltage getVoltage() {
        return inputs.leftMotor.motor.appliedVoltage;
    }

    public void setVoltage(Measure<VoltageUnit> voltage) {
        io.setVoltage(voltage);
    }
    
    public void setLength(Measure<DistanceUnit> length) {
        io.setLength(length);
    }
}
