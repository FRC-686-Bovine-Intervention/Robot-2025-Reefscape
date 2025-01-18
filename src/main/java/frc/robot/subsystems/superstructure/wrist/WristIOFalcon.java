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
import frc.robot.constants.HardwareDevices;

public class WristIOFalcon implements WristIO {
    private final TalonFX motor = HardwareDevices.wristMotorID.talonFX(); 
    private final CANcoder cancoder = HardwareDevices.wristEncoderID.cancoder();
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    
    public WristIOFalcon() {
        var cancoderConfig = new CANcoderConfiguration();

        cancoder.getConfigurator().apply(cancoderConfig);

        var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        ;
        motorConfig.Feedback
            .withRemoteCANcoder(cancoder)
            .withRotorToSensorRatio(WristConstants.motorToMechanism.ratio())
        ;
        motor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.encoder.updateFrom(cancoder);
        inputs.motor.updateFrom(motor);
    }

    @Override
    public void setVoltage(Measure<VoltageUnit> voltage) {
        motor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void setAngle(Measure<AngleUnit> angle) {
        motor.setControl(positionRequest.withPosition(angle.in(Rotations)));
    }
}

