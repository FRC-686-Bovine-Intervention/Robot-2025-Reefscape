package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.robot.constants.HardwareDevices;

public class ClimberIOTalonFXS implements ClimberIO {
    protected final TalonFXS motor = HardwareDevices.climberCageMotorID.talonFXS();

    public ClimberIOTalonFXS() {
        var motorConfig = new TalonFXSConfiguration();
        motorConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
        motorConfig.CurrentLimits
            .withStatorCurrentLimit(Amps.of(20))
            .withStatorCurrentLimitEnable(true);
        motor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.motor.updateFrom(motor);
    }

    @Override
    public void setMotorVoltage(Measure<VoltageUnit> voltage) {
        motor.setVoltage(voltage.magnitude());
    }
}
