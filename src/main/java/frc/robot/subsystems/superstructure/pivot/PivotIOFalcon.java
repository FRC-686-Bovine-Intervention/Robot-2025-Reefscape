package frc.robot.subsystems.superstructure.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

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

import edu.wpi.first.math.util.Units;
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
        DegreesPerSecond.of(90),
        DegreesPerSecondPerSecond.of(90)
    );
    private final LoggedTunableFF ffConsts = new LoggedTunableFF(
        "Pivot/FF",
        0,
        0,
        Units.rotationsToRadians(0.1),
        Units.rotationsToRadians(0.1)
    );
    private final LoggedTunablePID pidConsts = new LoggedTunablePID(
        "Pivot/PID",
        0.5,
        0,
        0
    );

    // Initial Configuration
    public PivotIOFalcon() {
        var encoderConfig = new CANcoderConfiguration();

        cancoder.getConfigurator().apply(encoderConfig);

        var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        ;
        motorConfig.Feedback
            .withRemoteCANcoder(cancoder)
            .withRotorToSensorRatio(PivotConstants.motorToMechanism.concat(PivotConstants.sensorToMechanism.inverse()).ratio())
            .withSensorToMechanismRatio(PivotConstants.sensorToMechanism.ratio())
        ;
        motorConfig.SoftwareLimitSwitch
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(Degrees.of(100))
            .withReverseSoftLimitThreshold(Degrees.of(0))
        ;

        profileConsts.update(motorConfig.MotionMagic);
        ffConsts.update(motorConfig.Slot0);
        pidConsts.update(motorConfig.Slot0);

        leftMotor.getConfigurator().apply(motorConfig);

        motorConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
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

    // Immediately stop
    @Override
    public void stop() {
        leftMotor.disable();
    }
}
