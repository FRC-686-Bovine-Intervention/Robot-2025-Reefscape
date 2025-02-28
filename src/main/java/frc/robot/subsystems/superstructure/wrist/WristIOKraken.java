package frc.robot.subsystems.superstructure.wrist;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

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
import frc.util.loggerUtil.tunables.LoggedTunableAngularProfile;
import frc.util.loggerUtil.tunables.LoggedTunableFF;
import frc.util.loggerUtil.tunables.LoggedTunablePID;

public class WristIOKraken implements WristIO {
    protected final TalonFX motor = HardwareDevices.wristMotorID.talonFX(); 
    protected final CANcoder cancoder = HardwareDevices.wristEncoderID.cancoder();
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private static final LoggedTunableAngularProfile profileConsts = new LoggedTunableAngularProfile(
        "Wrist/Profile",
        DegreesPerSecond.of(90),
        DegreesPerSecondPerSecond.of(90)
    );
    private static final LoggedTunableFF ffConsts = new LoggedTunableFF(
        "Wrist/FF",
        0,
        0,
        0,
        0
    );
    private static final LoggedTunablePID pidConsts = new LoggedTunablePID(
        "Wrist/PID",
        0,
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
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        ;
        motorConfig.Feedback
            .withRemoteCANcoder(cancoder)
            .withRotorToSensorRatio(WristConstants.motorToSensor.inverse().ratio())
            .withSensorToMechanismRatio(WristConstants.sensorToMechanism.inverse().ratio())
        ;
        profileConsts.update(motorConfig.MotionMagic);
        ffConsts.update(motorConfig.Slot0);
        pidConsts.update(motorConfig.Slot0);

        motor.getConfigurator().apply(motorConfig);
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

