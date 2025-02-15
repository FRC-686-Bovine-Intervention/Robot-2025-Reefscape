package frc.robot.subsystems.superstructure.pivot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;
import frc.util.robotStructure.angle.ArmMech;

public class Pivot {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    public final ArmMech mech = new ArmMech(PivotConstants.pivotBase);

    public static final LoggedTunableMeasure<VoltageUnit> climberMotorVoltage = new LoggedTunableMeasure<>("Pivot/Climber Motor Voltage", Volts.of(0));
    public static final LoggedTunableMeasure<VoltageUnit> pivotClimbingMotorVoltage = new LoggedTunableMeasure<>("Pivot/Pivot Climbing Motor Voltage", Volts.of(0));

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

    public Distance getChainLength(Angle angle) {
        return Meters.of(
            Math.sqrt(
                Math.pow(PivotConstants.pivotToChain.in(Meters), 2) +
                Math.pow(PivotConstants.pivotToClimberMotor.in(Meters), 2) -
                2 * PivotConstants.pivotToChain.in(Meters) * PivotConstants.pivotToClimberMotor.in(Meters) * Math.cos(angle.in(Radians))
            )
        );
    }

    public void setPivotPosition(Measure<AngleUnit> angle) {
        io.setPivotPosition(angle);
    }

    public void setPivotVoltage(Measure<VoltageUnit> voltage) {
        io.setPivotVoltage(voltage);
    }

    public void setClimberVoltage(Measure<VoltageUnit> voltage) {
        io.setClimberVoltage(voltage);
    }

    public void setPivotBrakeMode(boolean brake) {
        io.setPivotBrakeMode(brake);
    }

    public void stop() {
        io.stop();
    }
}
