package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;
import frc.util.FFConstants;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.faults.DeviceFaults;
import frc.util.faults.DeviceFaults.FaultType;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor.EncodedMotorStatusSignalCache;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class ClimberIOFalcon implements ClimberIO {
    protected final TalonFX motor = HardwareDevices.climberMotorID.talonFX();
    protected final Servo servo = HardwareDevices.climberServoPort.servo();
    protected final DigitalInput sensor = HardwareDevices.climberSensor.input();

    private final EncodedMotorStatusSignalCache motorStatusSignalCache;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final MotionMagicVoltage nonClimbingPositionRequest = new MotionMagicVoltage(0)
        .withSlot(0)
    ;
    private final MotionMagicVoltage climbingPositionRequest = new MotionMagicVoltage(0)
        .withSlot(1)
        .withOverrideBrakeDurNeutral(true)
        .withLimitForwardMotion(true)
    ;
    private final NeutralOut neutralOutRequest = new NeutralOut();
    private final CoastOut coastOutRequest = new CoastOut();
    private final StaticBrake staticBrakeRequest = new StaticBrake();

    private static final LoggedTunable<TrapezoidProfile.Constraints> profileConsts = LoggedTunable.fromDashboardUnits(
        "Climber/Profile",
        RotationsPerSecond,
        RotationsPerSecondPerSecond,
        RotationsPerSecond,
        RotationsPerSecondPerSecond,
        new TrapezoidProfile.Constraints(
            6,
            12
        )
    );
    private static final LoggedTunable<FFConstants> nonClimbingFFConsts = LoggedTunable.from(
        "Climber/Nonclimbing/FF",
        new FFConstants(
            0,
            0,
            0,
            0
        )
    );
    private static final LoggedTunable<PIDConstants> nonClimbingPIDConsts = LoggedTunable.from(
        "Climber/Nonclimbing/PID",
        new PIDConstants(
            8,
            0,
            0
        )
    );
    private static final LoggedTunable<FFConstants> climbingFFConsts = LoggedTunable.from(
        "Climber/Climbing/FF",
        new FFConstants(
            0,
            0,
            0,
            0
        )
    );
    private static final LoggedTunable<PIDConstants> climbingPIDConsts = LoggedTunable.from(
        "Climber/Climbing/PID",
        new PIDConstants(
            16,
            0,
            0
        )
    );


    public ClimberIOFalcon() {
        var motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
        ;
        motorConfig.HardwareLimitSwitch
            .withReverseLimitEnable(true)
            .withReverseLimitSource(ReverseLimitSourceValue.LimitSwitchPin)
            .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
            .withReverseLimitAutosetPositionEnable(true)
            .withReverseLimitAutosetPositionValue(ClimberConstants.climberMinimumAngle)
        ;

        motorConfig.Feedback
            .withSensorToMechanismRatio(ClimberConstants.sensorToMechanismRatio.reductionUnsigned())
        ;

        var profileConstraints = profileConsts.get();
        motorConfig.MotionMagic
            .withMotionMagicCruiseVelocity(profileConstraints.maxVelocity)
            .withMotionMagicAcceleration(profileConstraints.maxAcceleration)
        ;
        nonClimbingFFConsts.get().update(motorConfig.Slot0);
        nonClimbingPIDConsts.get().update(motorConfig.Slot0);
        climbingFFConsts.get().update(motorConfig.Slot1);
        climbingPIDConsts.get().update(motorConfig.Slot1);

        profileConsts.hasChanged(hashCode());
        nonClimbingFFConsts.hasChanged(hashCode());
        nonClimbingPIDConsts.hasChanged(hashCode());
        climbingFFConsts.hasChanged(hashCode());
        climbingPIDConsts.hasChanged(hashCode());

        this.motor.getConfigurator().apply(motorConfig);

        this.motorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.motor);

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.motorStatusSignalCache.encoder().getStatusSignals());
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(2), this.motorStatusSignalCache.motor().getStatusSignals());
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getFaultStatusSignals(this.motor));
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getStickyFaultStatusSignals(this.motor));
        this.motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            this.motorStatusSignalCache.encoder().position(),
            this.motorStatusSignalCache.encoder().velocity(),
            this.motorStatusSignalCache.motor().appliedVoltage(),
            this.motorStatusSignalCache.motor().statorCurrent(),
            this.motorStatusSignalCache.motor().deviceTemperature()
        );
        inputs.motorConnected = BaseStatusSignal.isAllGood(
            this.motorStatusSignalCache.encoder().position(),
            this.motorStatusSignalCache.encoder().velocity(),
            this.motorStatusSignalCache.motor().appliedVoltage(),
            this.motorStatusSignalCache.motor().statorCurrent(),
            this.motorStatusSignalCache.motor().deviceTemperature()
        );
        inputs.motor.updateFrom(this.motorStatusSignalCache);
        // inputs.motorFaults.updateFrom(this.motor);

        inputs.sensor = this.sensor.get() ^ ClimberConstants.climberSensorInverted;

        this.voltageRequest.withLimitReverseMotion(inputs.sensor);
        this.nonClimbingPositionRequest.withLimitReverseMotion(inputs.sensor);
        this.climbingPositionRequest.withLimitReverseMotion(inputs.sensor);

        if (profileConsts.hasChanged(this.hashCode())) {
            var config = new MotionMagicConfigs();
            this.motor.getConfigurator().refresh(config);
            var profileConstraints = profileConsts.get();
            config
                .withMotionMagicCruiseVelocity(profileConstraints.maxVelocity)
                .withMotionMagicAcceleration(profileConstraints.maxAcceleration)
            ;
            this.motor.getConfigurator().apply(config);
        }

        if (LoggedTunable.hasChanged(this.hashCode(), nonClimbingFFConsts, nonClimbingPIDConsts)) {
            var config = new Slot0Configs();
            this.motor.getConfigurator().refresh(config);
            nonClimbingFFConsts.get().update(config);
            nonClimbingPIDConsts.get().update(config);
            this.motor.getConfigurator().apply(config);
        }

        if (LoggedTunable.hasChanged(this.hashCode(), climbingFFConsts, climbingPIDConsts)) {
            var config = new Slot1Configs();
            this.motor.getConfigurator().refresh(config);
            climbingFFConsts.get().update(config);
            climbingPIDConsts.get().update(config);
            this.motor.getConfigurator().apply(config);
        }
    }

    @Override
    public void setVolts(double volts) {
        this.motor.setControl(this.voltageRequest
            .withOutput(volts)
        );
    }

    @Override
    public void setRatchetServoAngle(double angleRads) {
        this.servo.setAngle(
            Units.radiansToDegrees(angleRads)
        );
    }

    @Override
    public void setNonClimbingAngle(double angleRads) {
        this.motor.setControl(this.nonClimbingPositionRequest
            .withPosition(Units.radiansToRotations(angleRads))
        );
    }

    @Override
    public void setClimbingAngle(double angleRads) {
        this.motor.setControl(this.climbingPositionRequest
            .withPosition(Units.radiansToRotations(angleRads))
        );
    }

    @Override
    public void stop(Optional<NeutralMode> neutralMode) {
        var controlRequest = NeutralMode.selectControlRequest(neutralMode, this.neutralOutRequest, this.coastOutRequest, this.staticBrakeRequest);
        this.motor.setControl(controlRequest);
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
}
