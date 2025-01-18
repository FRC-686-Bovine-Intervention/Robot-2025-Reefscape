package frc.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;

public class WristIOFalcon implements WristIO {
    private final CANcoder cancoder = new CANcoder(0);
    private final TalonFX motor = new TalonFX(0); 
    private final MotionMagicVoltage profile = new MotionMagicVoltage(0);
    
    public WristIOFalcon() {
        var cancoderConfig = new CANcoderConfiguration();
        cancoder.getConfigurator().apply(cancoderConfig);
        var motorConfig = new TalonFXConfiguration(); 
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.Feedback.withRemoteCANcoder(cancoder);
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.Feedback.withRotorToSensorRatio(25);
        motor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void updateInputs (WristIOInputs inputs) {
        inputs.encoder.updateFrom(cancoder);
        inputs.motor.updateFrom(motor);
    }

    @Override
    public void setVoltage(Measure<VoltageUnit> voltage) {
        motor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void setAngle (Measure<AngleUnit> angle) {
        motor.setControl(profile.withPosition(angle.in(Rotations)));
    }


}

