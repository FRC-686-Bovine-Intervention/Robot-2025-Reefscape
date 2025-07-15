package frc.robot.subsystems.superstructure.elevator;

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

public class ElevatorIOKraken implements ElevatorIO {
    protected final TalonFX motor = HardwareDevices.elevatorMotorID.talonFX();
    protected final CANcoder cancoder = HardwareDevices.elevatorEncoderID.cancoder();

    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public ElevatorIOKraken() {
        var encoderConfig = new CANcoderConfiguration();
        cancoder.getConfigurator().refresh(encoderConfig.MagnetSensor);
        encoderConfig.MagnetSensor
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        ;

        cancoder.getConfigurator().apply(encoderConfig);

        var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        ;
        motorConfig.Feedback
            .withRemoteCANcoder(cancoder)
            .withRotorToSensorRatio(ElevatorConstants.motorToMechanism.then(ElevatorConstants.sensorToMechanism.inverse()).reductionUnsigned())
            .withSensorToMechanismRatio(ElevatorConstants.sensorToMechanism.reductionUnsigned())
        ;
        motorConfig.SoftwareLimitSwitch
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(ElevatorConstants.stage1LinearRelation.distanceToAngle(ElevatorConstants.minLengthPhysical.div(ElevatorConstants.movingStageCount)))
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(ElevatorConstants.stage1LinearRelation.distanceToAngle(ElevatorConstants.maxLengthSoftware.div(ElevatorConstants.movingStageCount)))
        ;

        motor.getConfigurator().apply(motorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
            RobotConstants.rioUpdateFrequency,
            motor.getRotorPosition(),
            motor.getRotorVelocity(),
            cancoder.getPosition(),
            cancoder.getVelocity()
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
            DriveConstants.odometryLoopFrequency.div(2),
            motor.getMotorVoltage(),
            motor.getStatorCurrent(),
            motor.getDeviceTemp()
        );
        motor.optimizeBusUtilization();
        cancoder.optimizeBusUtilization();
    }
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.encoder.updateFrom(cancoder);
        inputs.motor.updateFrom(motor);
    }

    @Override
    public void setVoltage(Measure<VoltageUnit> voltage) {
        motor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void setPosition(Measure<AngleUnit> position, Measure<AngularVelocityUnit> velocity, Measure<VoltageUnit> feedforward) {
        motor.setControl(positionRequest
            .withPosition(position.in(Rotations))
            .withVelocity(velocity.in(RotationsPerSecond))
            .withFeedForward(feedforward.in(Volts))
        );
    }

    @Override
    public void stop(Optional<NeutralMode> neutralMode) {
        motor.setControl(neutralMode.map(NeutralMode::getPhoenix6ControlRequest).orElseGet(NeutralOut::new));
    }

    @Override
    public void configPID(PIDConstants pidConstants) {
        var config = new Slot0Configs();
        motor.getConfigurator().refresh(config);
        pidConstants.update(config);
        motor.getConfigurator().apply(config);
    }
}
