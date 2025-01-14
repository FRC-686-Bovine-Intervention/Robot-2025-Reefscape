package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.robot.constants.HardwareDevices;


public class IntakeIOTalon implements IntakeIO{
    protected final TalonSRX motor = HardwareDevices.intakeMotorID.talonSRX();

    public IntakeIOTalon(){
        //Value needs review
        motor.configContinuousCurrentLimit(6);
    }

    @Override
    public void setMotorVoltage(Measure<VoltageUnit> volts) {
        //Value needs review
        motor.set(ControlMode.PercentOutput, volts.in(Volts) / 12);
    }
}
