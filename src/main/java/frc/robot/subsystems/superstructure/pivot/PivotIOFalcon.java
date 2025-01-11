package frc.robot.subsystems.superstructure.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

public class PivotIOFalcon implements PivotIO {
    private final TalonFX leftMotor = new TalonFX(0);
    private final TalonFX rightMotor = new TalonFX(1);
    private final MotionMagicVoltage profile = new MotionMagicVoltage(
        0
    );

    // Initial Configuration
    public PivotIOFalcon () {
        var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftMotor.getConfigurator().apply(motorConfig);
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightMotor.getConfigurator().apply(motorConfig);
        rightMotor.setControl(new StrictFollower(leftMotor.getDeviceID()));
    }

    public void updateInputs (PivotIOInputs inputs) {
        inputs.leftMotor.updateFrom(leftMotor);
        inputs.rightMotor.updateFrom(rightMotor);
    }

    // Set Voltage
    public void setPivotVoltage (double voltage) {
        leftMotor.setVoltage(voltage);
    }

    // Set position based on profile
    public void setPivotPosition (double goal) {
        leftMotor.setControl(profile.withPosition(Units.radiansToRotations(goal)));
    }

    // Immediately stop
    public void stop () {
        leftMotor.disable();
    }
}
