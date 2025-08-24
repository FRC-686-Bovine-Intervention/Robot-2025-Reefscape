package frc.robot.subsystems.superstructure.elevator;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.faults.DeviceFaults;
import frc.util.faults.DeviceFaults.FaultType;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor.EncodedMotorStatusSignalCache;
import frc.util.loggerUtil.inputs.LoggedEncoder.EncoderStatusSignalCache;

public class ElevatorIOKraken implements ElevatorIO {
    protected final TalonFX motor = HardwareDevices.elevatorMotorID.talonFX();
    protected final CANcoder cancoder = HardwareDevices.elevatorEncoderID.cancoder();

    private final EncodedMotorStatusSignalCache motorStatusSignalCache;
    private final EncoderStatusSignalCache encoderStatusSignalCache;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final NeutralOut neutralOutRequest = new NeutralOut();
    private final CoastOut coastOutRequest = new CoastOut();
    private final StaticBrake staticBrakeRequest = new StaticBrake();

    public ElevatorIOKraken() {
        var encoderConfig = new CANcoderConfiguration();
        this.cancoder.getConfigurator().refresh(encoderConfig.MagnetSensor);
        encoderConfig.MagnetSensor
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        ;

        this.cancoder.getConfigurator().apply(encoderConfig);

        var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        ;
        motorConfig.Feedback
            .withRemoteCANcoder(this.cancoder)
            .withRotorToSensorRatio(ElevatorConstants.motorToMechanism.then(ElevatorConstants.sensorToMechanism.inverse()).reductionUnsigned())
            .withSensorToMechanismRatio(ElevatorConstants.sensorToMechanism.reductionUnsigned())
        ;
        motorConfig.SoftwareLimitSwitch
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(ElevatorConstants.stage1LinearRelation.distanceToAngle(ElevatorConstants.minLengthPhysical.div(ElevatorConstants.movingStageCount)))
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(ElevatorConstants.stage1LinearRelation.distanceToAngle(ElevatorConstants.maxLengthSoftware.div(ElevatorConstants.movingStageCount)))
        ;

        this.motor.getConfigurator().apply(motorConfig);

        this.motorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.motor);
        this.encoderStatusSignalCache = EncoderStatusSignalCache.from(this.cancoder);

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.motorStatusSignalCache.encoder().getStatusSignals());
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.encoderStatusSignalCache.getStatusSignals());
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(2), this.motorStatusSignalCache.motor().getStatusSignals());
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getFaultStatusSignals(this.motor));
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getStickyFaultStatusSignals(this.motor));
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getFaultStatusSignals(this.cancoder));
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getStickyFaultStatusSignals(this.cancoder));
        this.motor.optimizeBusUtilization();
        this.cancoder.optimizeBusUtilization();
    }
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            this.encoderStatusSignalCache.position(),
            this.encoderStatusSignalCache.velocity(),
            this.motorStatusSignalCache.encoder().position(),
            this.motorStatusSignalCache.encoder().velocity(),
            this.motorStatusSignalCache.motor().appliedVoltage(),
            this.motorStatusSignalCache.motor().statorCurrent(),
            this.motorStatusSignalCache.motor().deviceTemperature()
        );
        inputs.encoder.updateFrom(this.encoderStatusSignalCache);
        inputs.motor.updateFrom(this.motorStatusSignalCache);
        // inputs.encoderFaults.updateFrom(this.cancoder);
        // inputs.motorFaults.updateFrom(this.motor);
    }

    @Override
    public void setVolts(double volts) {
        this.motor.setControl(this.voltageRequest
            .withOutput(volts)
        );
    }

    @Override
    public void setPosition(double positionRads, double velocityRadsPerSec, double feedforwardVolts) {
        this.motor.setControl(this.positionRequest
            .withPosition(Units.radiansToRotations(positionRads))
            .withVelocity(Units.radiansToRotations(velocityRadsPerSec))
            .withFeedForward(feedforwardVolts)
        );
    }

    @Override
    public void stop(Optional<NeutralMode> neutralMode) {
        var controlRequest = NeutralMode.selectControlRequest(neutralMode, this.neutralOutRequest, this.coastOutRequest, this.staticBrakeRequest);
        this.motor.setControl(controlRequest);
    }

    @Override
    public void configPID(PIDConstants pidConstants) {
        var config = new Slot0Configs();
        this.motor.getConfigurator().refresh(config);
        pidConstants.update(config);
        this.motor.getConfigurator().apply(config);
    }

    @Override
    public void clearMotorStickyFaults(long bitmask) {
        if (bitmask == DeviceFaults.noneMask) {return;}
        if (bitmask == DeviceFaults.allMask) {
            this.motor.clearStickyFaults();
        } else {
            for (var faultType : FaultType.possibleTalonFXFaults) {
                if (faultType.isPartOf(bitmask)) {
                    faultType.clearStickyFaultOn(this.motor);
                }
            }
        }
    }
    @Override
    public void clearEncoderStickyFaults(long bitmask) {
        if (bitmask == DeviceFaults.noneMask) {return;}
        if (bitmask == DeviceFaults.allMask) {
            this.cancoder.clearStickyFaults();
        } else {
            for (var faultType : FaultType.possibleCancoderFaults) {
                if (faultType.isPartOf(bitmask)) {
                    faultType.clearStickyFaultOn(this.cancoder);
                }
            }
        }
    }
}
