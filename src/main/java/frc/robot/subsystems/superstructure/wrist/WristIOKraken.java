package frc.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.util.loggerUtil.tunables.LoggedTunableAngularProfile;
import frc.util.loggerUtil.tunables.LoggedTunableFF;
import frc.util.loggerUtil.tunables.LoggedTunablePID;

public class WristIOKraken implements WristIO {
    protected final TalonFX motor = HardwareDevices.wristMotorID.talonFX(); 
    protected final CANcoder cancoder = HardwareDevices.wristEncoderID.cancoder();
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private static final LoggedTunableAngularProfile profileConsts = new LoggedTunableAngularProfile(
        "Wrist/Profile",
        DegreesPerSecond.of(360),
        DegreesPerSecondPerSecond.of(720)
    );
    private static final LoggedTunableFF ffConsts = new LoggedTunableFF(
        "Wrist/FF",
        0,
        0,
        5,
        0
    );
    private static final LoggedTunablePID pidConsts = new LoggedTunablePID(
        "Wrist/PID",
        50,
        0,
        0
    );
    
    public WristIOKraken() {
        var cancoderConfig = new CANcoderConfiguration();

        cancoder.getConfigurator().refresh(cancoderConfig.MagnetSensor);
        cancoderConfig.MagnetSensor
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        ;

        cancoder.getConfigurator().apply(cancoderConfig);

        var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        ;
        motorConfig.Feedback
            .withRemoteCANcoder(cancoder)
            .withRotorToSensorRatio(WristConstants.motorToSensor.inverse().ratio())
            .withSensorToMechanismRatio(WristConstants.sensorToMechanism.inverse().ratio())
        ;
        motorConfig.SoftwareLimitSwitch
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(WristConstants.minAngle)
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(WristConstants.maxAngle)
        ;

        profileConsts.update(motorConfig.MotionMagic);
        ffConsts.update(motorConfig.Slot0);
        pidConsts.update(motorConfig.Slot0);

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
    public void updateInputs(WristIOInputs inputs) {
        inputs.encoder.updateFrom(cancoder);
        inputs.motor.updateFrom(motor);

        if (profileConsts.hasChanged(hashCode())) {
            var config = new MotionMagicConfigs();
            motor.getConfigurator().refresh(config);
            profileConsts.update(config);
            motor.getConfigurator().apply(config);
        }

        if (ffConsts.hasChanged(hashCode()) | pidConsts.hasChanged(hashCode())) {
            var config = new Slot0Configs();
            motor.getConfigurator().refresh(config);
            ffConsts.update(config);
            pidConsts.update(config);
            motor.getConfigurator().apply(config);
        }

        // Logger.recordOutput("Superstructure/Wrist/Motor/posiion", motor.getPosition().getValueAsDouble());
        // Logger.recordOutput("Superstructure/Wrist/Motor/veloctiy", motor.getVelocity().getValueAsDouble());
        // Logger.recordOutput("Superstructure/Wrist/Motor/Profile/Position", motor.getClosedLoopReference().getValueAsDouble());
        // Logger.recordOutput("Superstructure/Wrist/Motor/Profile/Velocity", motor.getClosedLoopReferenceSlope().getValueAsDouble());
        // Logger.recordOutput("Superstructure/Wrist/Motor/PID error", motor.getClosedLoopError().getValueAsDouble());
        // Logger.recordOutput("Superstructure/Wrist/Motor/Out", motor.getClosedLoopOutput().getValueAsDouble());
    }

    @Override
    public void setVoltage(Measure<VoltageUnit> voltage) {
        motor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void setAngle(Measure<AngleUnit> angle) {
        motor.setControl(positionRequest.withPosition(angle.in(Rotations)));
    }
    
    @Override
    public void setFeedForward(Measure<VoltageUnit> feedForward) {
        positionRequest.withFeedForward(feedForward.in(Volts));
    }
}

