package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;

public class ElevatorIOFalcon implements ElevatorIO {
    private TalonFX leftMotor = new TalonFX(0);
    private TalonFX rightMotor = new TalonFX(1);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(
        0
    );
    
    public ElevatorIOFalcon() {
         var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        ;
        leftMotor.getConfigurator().apply(motorConfig);
        motorConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        rightMotor.getConfigurator().apply(motorConfig);
        rightMotor.setControl(new StrictFollower(leftMotor.getDeviceID()));
    }
    
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leftMotor.updateFrom(leftMotor);
        inputs.rightMotor.updateFrom(rightMotor);
    }

    @Override
    public void setVoltage(Measure<VoltageUnit> voltage) {
        leftMotor.setVoltage(voltage.baseUnitMagnitude());
    }

    @Override
    public void setPosition(Measure<DistanceUnit> dist) {
        leftMotor.setControl(positionRequest.withPosition(Radians.of(dist.in(Inches)/1)));
    }
}
