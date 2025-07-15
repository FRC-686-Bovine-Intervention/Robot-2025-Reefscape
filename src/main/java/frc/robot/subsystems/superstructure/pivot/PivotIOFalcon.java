package frc.robot.subsystems.superstructure.pivot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.util.NeutralMode;
import frc.util.PIDConstants;

public class PivotIOFalcon implements PivotIO {
    protected final TalonFX leftMotor = HardwareDevices.pivotLeftMotorID.talonFX();
    protected final TalonFX rightMotor = HardwareDevices.pivotRightMotorID.talonFX();
    protected final CANcoder cancoder = HardwareDevices.pivotEncoderID.cancoder();

    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public PivotIOFalcon() {
        var encoderConfig = new CANcoderConfiguration();

        cancoder.getConfigurator().refresh(encoderConfig.MagnetSensor);
        encoderConfig.MagnetSensor
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        ;

        cancoder.getConfigurator().apply(encoderConfig);

        var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        ;
        motorConfig.Feedback
            .withRemoteCANcoder(cancoder)
            .withRotorToSensorRatio(PivotConstants.motorToMechanism.then(PivotConstants.sensorToMechanism.inverse()).reductionUnsigned())
            .withSensorToMechanismRatio(PivotConstants.sensorToMechanism.reductionUnsigned())
        ;
        motorConfig.SoftwareLimitSwitch
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(PivotConstants.minAngle)
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(PivotConstants.maxAngle)
        ;

        leftMotor.getConfigurator().apply(motorConfig);

        motorConfig.MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive)
        ;
        rightMotor.getConfigurator().apply(motorConfig);
        rightMotor.setControl(new StrictFollower(leftMotor.getDeviceID()));

        BaseStatusSignal.setUpdateFrequencyForAll(
            RobotConstants.rioUpdateFrequency,
            leftMotor.getRotorPosition(),
            leftMotor.getRotorVelocity(),
            rightMotor.getRotorPosition(),
            rightMotor.getRotorVelocity(),
            cancoder.getPosition(),
            cancoder.getVelocity()
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
            DriveConstants.odometryLoopFrequency.div(2),
            leftMotor.getMotorVoltage(),
            leftMotor.getStatorCurrent(),
            leftMotor.getDeviceTemp(),
            rightMotor.getMotorVoltage(),
            rightMotor.getStatorCurrent(),
            rightMotor.getDeviceTemp()
        );
        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
        cancoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.encoder.updateFrom(cancoder);
        inputs.leftMotor.updateFrom(leftMotor);
        inputs.rightMotor.updateFrom(rightMotor);
    }

    @Override
    public void setVoltage(Measure<VoltageUnit> voltage) {
        leftMotor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void setPosition(Measure<AngleUnit> position, Measure<AngularVelocityUnit> velocity, Measure<VoltageUnit> feedforward) {
        leftMotor.setControl(positionRequest
            .withPosition(position.in(Rotations))
            .withVelocity(velocity.in(RotationsPerSecond))
            .withFeedForward(feedforward.in(Volts))
        );
    }
    
    @Override
    public void stop(Optional<NeutralMode> neutralMode) {
        leftMotor.setControl(neutralMode.map(NeutralMode::getPhoenix6ControlRequest).orElseGet(NeutralOut::new));
    }

    @Override
    public void configPID(PIDConstants pidConstants) {
        var leftConfig = new Slot0Configs();
        var rightConfig = new Slot0Configs();
        leftMotor.getConfigurator().refresh(leftConfig);
        rightMotor.getConfigurator().refresh(rightConfig);
        pidConstants.update(leftConfig);
        pidConstants.update(rightConfig);
        leftMotor.getConfigurator().apply(leftConfig);
        rightMotor.getConfigurator().apply(rightConfig);
    }
}
