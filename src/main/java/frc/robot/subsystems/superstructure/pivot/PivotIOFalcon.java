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
    private final StrictFollower followerRequest;

    public PivotIOFalcon() {
        var encoderConfig = new CANcoderConfiguration();

        this.cancoder.getConfigurator().refresh(encoderConfig.MagnetSensor);
        encoderConfig.MagnetSensor
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        ;

        this.cancoder.getConfigurator().apply(encoderConfig);

        var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        ;
        motorConfig.Feedback
            .withRemoteCANcoder(this.cancoder)
            .withRotorToSensorRatio(PivotConstants.motorToMechanism.then(PivotConstants.sensorToMechanism.inverse()).reductionUnsigned())
            .withSensorToMechanismRatio(PivotConstants.sensorToMechanism.reductionUnsigned())
        ;
        motorConfig.SoftwareLimitSwitch
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(PivotConstants.minAngle)
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(PivotConstants.maxAngle)
        ;

        this.leftMotor.getConfigurator().apply(motorConfig);

        motorConfig.MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive)
        ;
        this.rightMotor.getConfigurator().apply(motorConfig);
        this.followerRequest = new StrictFollower(this.leftMotor.getDeviceID());
        this.rightMotor.setControl(this.followerRequest);

        BaseStatusSignal.setUpdateFrequencyForAll(
            RobotConstants.rioUpdateFrequency,
            this.leftMotor.getRotorPosition(),
            this.leftMotor.getRotorVelocity(),
            this.rightMotor.getRotorPosition(),
            this.rightMotor.getRotorVelocity(),
            this.cancoder.getPosition(),
            this.cancoder.getVelocity()
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
            DriveConstants.odometryLoopFrequency.div(2),
            this.leftMotor.getMotorVoltage(),
            this.leftMotor.getStatorCurrent(),
            this.leftMotor.getDeviceTemp(),
            this.rightMotor.getMotorVoltage(),
            this.rightMotor.getStatorCurrent(),
            this.rightMotor.getDeviceTemp()
        );
        this.leftMotor.optimizeBusUtilization();
        this.rightMotor.optimizeBusUtilization();
        this.cancoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.encoder.updateFrom(this.cancoder);
        inputs.leftMotor.updateFrom(this.leftMotor);
        inputs.rightMotor.updateFrom(this.rightMotor);
    }

    @Override
    public void setVoltage(Measure<VoltageUnit> voltage) {
        this.leftMotor.setVoltage(voltage.in(Volts));
        this.rightMotor.setControl(this.followerRequest);
    }

    @Override
    public void setPosition(Measure<AngleUnit> position, Measure<AngularVelocityUnit> velocity, Measure<VoltageUnit> feedforward) {
        this.leftMotor.setControl(this.positionRequest
            .withPosition(position.in(Rotations))
            .withVelocity(velocity.in(RotationsPerSecond))
            .withFeedForward(feedforward.in(Volts))
        );
        this.rightMotor.setControl(this.followerRequest);
    }
    
    @Override
    public void stop(Optional<NeutralMode> neutralMode) {
        this.leftMotor.setControl(neutralMode.map(NeutralMode::getPhoenix6ControlRequest).orElseGet(NeutralOut::new));
        this.rightMotor.setControl(neutralMode.map(NeutralMode::getPhoenix6ControlRequest).orElseGet(NeutralOut::new));
    }

    @Override
    public void configPID(PIDConstants pidConstants) {
        var leftConfig = new Slot0Configs();
        var rightConfig = new Slot0Configs();
        this.leftMotor.getConfigurator().refresh(leftConfig);
        this.rightMotor.getConfigurator().refresh(rightConfig);
        pidConstants.update(leftConfig);
        pidConstants.update(rightConfig);
        this.leftMotor.getConfigurator().apply(leftConfig);
        this.rightMotor.getConfigurator().apply(rightConfig);
    }
}
