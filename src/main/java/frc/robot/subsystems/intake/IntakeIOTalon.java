package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.HardwareDevices;


public class IntakeIOTalon implements IntakeIO {
    protected final TalonSRX motor = HardwareDevices.intakeMotorID.talonSRX();
    protected final DigitalInput coralSensor = HardwareDevices.coralSensor.input();
    protected final DigitalInput algaeSensor = HardwareDevices.algaeSensor.input();

    public IntakeIOTalon() {
        motor.configContinuousCurrentLimit(20);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.motor.updateFrom(motor);

        inputs.coralSensor = coralSensor.get() ^ IntakeConstants.coralSensorInverted;
        inputs.algaeSensor = algaeSensor.get() ^ IntakeConstants.algaeSensorInverted;
    }
    @Override
    public void setMotorVoltage(Measure<VoltageUnit> volts) {
        motor.set(ControlMode.PercentOutput, volts.in(Volts) / 12);
    }
}
