package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.robot.constants.HardwareDevices;

public class ClimberIOFalcon implements ClimberIO {
    protected final TalonFX chainMotor = HardwareDevices.climberChainMotorID.talonFX();

    private final VoltageOut coastVoltage = new VoltageOut(0).withOverrideBrakeDurNeutral(false);
    private final VoltageOut brakeVoltage = new VoltageOut(0).withOverrideBrakeDurNeutral(true);

    public ClimberIOFalcon() {
        var motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
        ;

        chainMotor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.chainMotor.updateFrom(chainMotor);
    }

    @Override
    public void setCoastVoltage(Measure<VoltageUnit> voltage) {
        chainMotor.setControl(coastVoltage.withOutput(voltage.in(Volts)));
    }

    @Override
    public void setBrakeVoltage(Measure<VoltageUnit> voltage) {
        chainMotor.setControl(brakeVoltage.withOutput(voltage.in(Volts)));
    }
}
