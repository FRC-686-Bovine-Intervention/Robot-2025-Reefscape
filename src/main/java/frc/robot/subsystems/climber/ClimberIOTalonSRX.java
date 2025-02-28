package frc.robot.subsystems.climber;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.robot.constants.HardwareDevices;

public class ClimberIOTalonSRX implements ClimberIO {
    protected final TalonSRX motor = HardwareDevices.climberMotorID.talonSRX();

    public ClimberIOTalonSRX() {
        motor.configContinuousCurrentLimit(20);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.motor.updateFrom(motor);
    }

    @Override
    public void setMotorVoltage(Measure<VoltageUnit> voltage) {
        motor.set(ControlMode.PercentOutput, voltage.in(Volts) / 12);
    }
}
