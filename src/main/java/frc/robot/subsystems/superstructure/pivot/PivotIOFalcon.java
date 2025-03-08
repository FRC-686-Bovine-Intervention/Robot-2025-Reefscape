package frc.robot.subsystems.superstructure.pivot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.robot.constants.HardwareDevices;
import frc.util.loggerUtil.tunables.LoggedTunableAngularProfile;
import frc.util.loggerUtil.tunables.LoggedTunableFF;
import frc.util.loggerUtil.tunables.LoggedTunablePID;

public class PivotIOFalcon implements PivotIO {
    protected final TalonFX leftMotor = HardwareDevices.pivotLeftMotorID.talonFX();
    protected final TalonFX rightMotor = HardwareDevices.pivotRightMotorID.talonFX();
    protected final CANcoder cancoder = HardwareDevices.pivotEncoderID.cancoder();

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

    private final LoggedTunableAngularProfile profileConsts = new LoggedTunableAngularProfile(
        "Pivot/Profile",
        DegreesPerSecond.of(135),
        DegreesPerSecondPerSecond.of(180)
    );
    private final LoggedTunableFF ffConsts = new LoggedTunableFF(
        "Pivot/FF",
        0,
        0,
        17,
        0
    );
    private final LoggedTunablePID pidConsts = new LoggedTunablePID(
        "Pivot/PID",
        150,
        0,
        0
    );

    // Initial Configuration
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
            .withRotorToSensorRatio(PivotConstants.motorToMechanism.concat(PivotConstants.sensorToMechanism.inverse()).inverse().ratio())
            .withSensorToMechanismRatio(PivotConstants.sensorToMechanism.inverse().ratio())
        ;
        motorConfig.SoftwareLimitSwitch
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(PivotConstants.minAngle)
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(PivotConstants.maxAngle)
        ;

        profileConsts.update(motorConfig.MotionMagic);
        ffConsts.update(motorConfig.Slot0);
        pidConsts.update(motorConfig.Slot0);

        leftMotor.getConfigurator().apply(motorConfig);

        motorConfig.MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive)
        ;
        rightMotor.getConfigurator().apply(motorConfig);
        rightMotor.setControl(new StrictFollower(leftMotor.getDeviceID()));
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.encoder.updateFrom(cancoder);
        inputs.leftMotor.updateFrom(leftMotor);
        inputs.rightMotor.updateFrom(rightMotor);

        if (profileConsts.hasChanged(hashCode())) {
            var config = new MotionMagicConfigs();
            leftMotor.getConfigurator().refresh(config);
            profileConsts.update(config);
            leftMotor.getConfigurator().apply(config);
        }
        if (ffConsts.hasChanged(hashCode()) | pidConsts.hasChanged(hashCode())) {
            var config = new Slot0Configs();
            leftMotor.getConfigurator().refresh(config);
            ffConsts.update(config);
            pidConsts.update(config);
            leftMotor.getConfigurator().apply(config);
        }

        Logger.recordOutput("Superstructure/Pivot/Motor/posiion", leftMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Superstructure/Pivot/Motor/veloctiy", leftMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Superstructure/Pivot/Motor/Profile/Position", leftMotor.getClosedLoopReference().getValueAsDouble());
        Logger.recordOutput("Superstructure/Pivot/Motor/Profile/Velocity", leftMotor.getClosedLoopReferenceSlope().getValueAsDouble());
        Logger.recordOutput("Superstructure/Pivot/Motor/PID error", leftMotor.getClosedLoopError().getValueAsDouble());
        Logger.recordOutput("Superstructure/Pivot/Motor/Out", leftMotor.getClosedLoopOutput().getValueAsDouble());
    }

    // Set Voltage
    @Override
    public void setVoltage(Measure<VoltageUnit> voltage) {
        leftMotor.setVoltage(voltage.in(Volts));
    }

    // Set position based on profile
    @Override
    public void setPosition(Measure<AngleUnit> position) {
        leftMotor.setControl(positionRequest.withPosition(position.in(Rotations)));
    }
    
    @Override
    public void setFeedForward(Measure<VoltageUnit> feedForward) {
        positionRequest.withFeedForward(feedForward.in(Volts));
    }

    // Immediately stop
    @Override
    public void stop() {
        leftMotor.disable();
    }
}
