package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.HardwareDevices;


public class IntakeIOFalcon implements IntakeIO{
    protected final TalonFX motor = HardwareDevices.intakeMotorID.talonFX();
    protected final DigitalInput sensor = new DigitalInput(9);

    public IntakeIOFalcon(){
        var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast).withInverted(InvertedValue.CounterClockwise_Positive);
        motorConfig.CurrentLimits.withStatorCurrentLimit(20).withStatorCurrentLimitEnable(true);

        motor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.motor.updateFrom(motor);

        inputs.sensorDetect = !sensor.get();
    }
    @Override
    public void setMotorVoltage(Measure<VoltageUnit> voltage) {
        motor.setVoltage(voltage.in(Volts));
    }
}
