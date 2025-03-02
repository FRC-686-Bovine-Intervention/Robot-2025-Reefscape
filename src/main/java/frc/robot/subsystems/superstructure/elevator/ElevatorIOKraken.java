package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.robot.constants.HardwareDevices;
import frc.util.loggerUtil.tunables.LoggedTunableFF;
import frc.util.loggerUtil.tunables.LoggedTunableLinearProfile;
import frc.util.loggerUtil.tunables.LoggedTunablePID;

public class ElevatorIOKraken implements ElevatorIO {
    protected final TalonFX motor = HardwareDevices.elevatorMotorID.talonFX();
    protected final CANcoder cancoder = HardwareDevices.elevatorEncoderID.cancoder();

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

    private final LoggedTunableLinearProfile profileConsts = new LoggedTunableLinearProfile(
        "Elevator/Profile",
        InchesPerSecond.of(12),
        InchesPerSecond.per(Second).of(24)
    );
    private final LoggedTunableFF ffConsts = new LoggedTunableFF(
        "Elevator/FF",
        0.2,
        0.3,
        2,
        0
    );
    private final LoggedTunablePID pidConsts = new LoggedTunablePID(
        "Elevator/PID",
        20,
        0,
        0
    );
    
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
            .withRotorToSensorRatio(-ElevatorConstants.motorToMechanism.concat(ElevatorConstants.sensorToMechanism.inverse()).inverse().ratio())
            .withSensorToMechanismRatio(-ElevatorConstants.sensorToMechanism.inverse().ratio())
        ;
        motorConfig.SoftwareLimitSwitch
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(Degrees.of(0))
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(Radians.of(ElevatorConstants.stageExtension.div(ElevatorConstants.sprocketRadius).baseUnitMagnitude()))
        ;
        motorConfig.Slot0
            .withGravityType(GravityTypeValue.Elevator_Static)
        ;

        profileConsts.update(motorConfig.MotionMagic, ElevatorConstants.sprocketRadius);
        ffConsts.update(motorConfig.Slot0);
        pidConsts.update(motorConfig.Slot0);

        motor.getConfigurator().apply(motorConfig);
    }
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.encoder.updateFrom(cancoder);
        inputs.motor.updateFrom(motor);

        if (profileConsts.hasChanged(hashCode())) {
            var config = new MotionMagicConfigs();
            motor.getConfigurator().refresh(config);
            profileConsts.update(config, ElevatorConstants.sprocketRadius);
            motor.getConfigurator().apply(config);
        }
        if (ffConsts.hasChanged(hashCode()) | pidConsts.hasChanged(hashCode())) {
            var config = new Slot0Configs();
            motor.getConfigurator().refresh(config);
            ffConsts.update(config);
            pidConsts.update(config);
            motor.getConfigurator().apply(config);
        }

        Logger.recordOutput("Superstructure/Elevator/Motor/posiion", motor.getPosition().getValueAsDouble());
        Logger.recordOutput("Superstructure/Elevator/Motor/veloctiy", motor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Superstructure/Elevator/Motor/Profile/Position", motor.getClosedLoopReference().getValueAsDouble());
        Logger.recordOutput("Superstructure/Elevator/Motor/Profile/Velocity", motor.getClosedLoopReferenceSlope().getValueAsDouble());
        Logger.recordOutput("Superstructure/Elevator/Motor/PID error", motor.getClosedLoopError().getValueAsDouble());
        Logger.recordOutput("Superstructure/Elevator/Motor/Out", motor.getClosedLoopOutput().getValueAsDouble());
    }

    @Override
    public void setVoltage(Measure<VoltageUnit> voltage) {
        motor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void setLength(Measure<DistanceUnit> length) {
        motor.setControl(positionRequest.withPosition(Radians.of(length.div(ElevatorConstants.sprocketRadius).baseUnitMagnitude() / ElevatorConstants.movingStageCount)));
    }

    @Override
    public void setFeedForward(Measure<VoltageUnit> feedForward) {
        positionRequest.withFeedForward(feedForward.in(Volts));
    }
}
