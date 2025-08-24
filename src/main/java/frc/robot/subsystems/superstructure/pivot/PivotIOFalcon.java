package frc.robot.subsystems.superstructure.pivot;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.StrictFollower;
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

public class PivotIOFalcon implements PivotIO {
    protected final TalonFX leftMotor = HardwareDevices.pivotLeftMotorID.talonFX();
    protected final TalonFX rightMotor = HardwareDevices.pivotRightMotorID.talonFX();
    protected final CANcoder cancoder = HardwareDevices.pivotEncoderID.cancoder();

    private final EncodedMotorStatusSignalCache leftMotorStatusSignalCache;
    private final EncodedMotorStatusSignalCache rightMotorStatusSignalCache;
    private final EncoderStatusSignalCache encoderStatusSignalCache;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);
    private final NeutralOut neutralOutRequest = new NeutralOut();
    private final CoastOut coastOutRequest = new CoastOut();
    private final StaticBrake staticBrakeRequest = new StaticBrake();
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

        this.leftMotorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.leftMotor);
        this.rightMotorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.rightMotor);
        this.encoderStatusSignalCache = EncoderStatusSignalCache.from(this.cancoder);

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.leftMotorStatusSignalCache.encoder().getStatusSignals());
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.rightMotorStatusSignalCache.encoder().getStatusSignals());
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.encoderStatusSignalCache.getStatusSignals());
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(2), this.leftMotorStatusSignalCache.motor().getStatusSignals());
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(2), this.rightMotorStatusSignalCache.motor().getStatusSignals());
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getFaultStatusSignals(this.leftMotor));
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getStickyFaultStatusSignals(this.leftMotor));
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getFaultStatusSignals(this.rightMotor));
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getStickyFaultStatusSignals(this.rightMotor));
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getFaultStatusSignals(this.cancoder));
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getStickyFaultStatusSignals(this.cancoder));
        this.leftMotor.optimizeBusUtilization();
        this.rightMotor.optimizeBusUtilization();
        this.cancoder.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            this.leftMotorStatusSignalCache.encoder().position(),
            this.leftMotorStatusSignalCache.encoder().velocity(),
            this.leftMotorStatusSignalCache.motor().appliedVoltage(),
            this.leftMotorStatusSignalCache.motor().statorCurrent(),
            this.leftMotorStatusSignalCache.motor().deviceTemperature(),
            this.rightMotorStatusSignalCache.encoder().position(),
            this.rightMotorStatusSignalCache.encoder().velocity(),
            this.rightMotorStatusSignalCache.motor().appliedVoltage(),
            this.rightMotorStatusSignalCache.motor().statorCurrent(),
            this.rightMotorStatusSignalCache.motor().deviceTemperature(),
            this.encoderStatusSignalCache.position(),
            this.encoderStatusSignalCache.velocity()
        );
        inputs.encoder.updateFrom(this.encoderStatusSignalCache);
        inputs.leftMotor.updateFrom(this.leftMotorStatusSignalCache);
        inputs.rightMotor.updateFrom(this.rightMotorStatusSignalCache);
        // inputs.encoderFaults.updateFrom(this.cancoder);
        // inputs.leftMotorFaults.updateFrom(this.leftMotor);
        // inputs.rightMotorFaults.updateFrom(this.rightMotor);
    }

    @Override
    public void setVolts(double volts) {
        this.leftMotor.setControl(this.voltageRequest
            .withOutput(volts)
        );
        this.rightMotor.setControl(this.followerRequest);
    }

    @Override
    public void setPosition(double positionRads, double velocityRadsPerSec, double feedforwardVolts) {
        this.leftMotor.setControl(this.positionRequest
            .withPosition(Units.radiansToRotations(positionRads))
            .withVelocity(Units.radiansToRotations(velocityRadsPerSec))
            .withFeedForward(feedforwardVolts)
        );
        this.rightMotor.setControl(this.followerRequest);
    }
    
    @Override
    public void stop(Optional<NeutralMode> neutralMode) {
        var controlRequest = NeutralMode.selectControlRequest(neutralMode, this.neutralOutRequest, this.coastOutRequest, this.staticBrakeRequest);
        this.leftMotor.setControl(controlRequest);
        this.rightMotor.setControl(controlRequest);
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

    @Override
    public void clearLeftMotorStickyFaults(long bitmask) {
        if (bitmask == DeviceFaults.noneMask) {return;}
        if (bitmask == DeviceFaults.allMask) {
            this.leftMotor.clearStickyFaults();
        } else {
            for (var faultType : FaultType.possibleTalonFXFaults) {
                if (faultType.isPartOf(bitmask)) {
                    faultType.clearStickyFaultOn(this.leftMotor);
                }
            }
        }
    }
    @Override
    public void clearRightMotorStickyFaults(long bitmask) {
        if (bitmask == DeviceFaults.noneMask) {return;}
        if (bitmask == DeviceFaults.allMask) {
            this.rightMotor.clearStickyFaults();
        } else {
            for (var faultType : FaultType.possibleTalonFXFaults) {
                if (faultType.isPartOf(bitmask)) {
                    faultType.clearStickyFaultOn(this.rightMotor);
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
