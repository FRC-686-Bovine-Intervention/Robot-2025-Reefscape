package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
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

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.robot.constants.HardwareDevices;
import frc.util.loggerUtil.tunables.LoggedTunableFF;
import frc.util.loggerUtil.tunables.LoggedTunableLinearProfile;
import frc.util.loggerUtil.tunables.LoggedTunablePID;

public class ElevatorIOFalcon implements ElevatorIO {
    protected final TalonFX leftMotor = HardwareDevices.elevatorLeftMotorID.talonFX();
    protected final TalonFX rightMotor = HardwareDevices.elevatorRightMotorID.talonFX();
    protected final CANcoder cancoder = HardwareDevices.elevatorEncoderID.cancoder();

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

    private final LoggedTunableLinearProfile profileConsts = new LoggedTunableLinearProfile(
        "Elevator/Profile",
        InchesPerSecond.of(17),
        InchesPerSecond.per(Second).of(34)
    );
    private final LoggedTunableFF ffConsts = new LoggedTunableFF(
        "Elevator/FF",
        0,
        0,
        0,
        0
    );
    private final LoggedTunablePID pidConsts = new LoggedTunablePID(
        "Elevator/PID",
        0,
        0,
        0
    );
    
    public ElevatorIOFalcon() {
        var encoderConfig = new CANcoderConfiguration();

        cancoder.getConfigurator().apply(encoderConfig);

        var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        ;
        motorConfig.Feedback
            .withRemoteCANcoder(cancoder)
            .withRotorToSensorRatio(100)
        ;
        motorConfig.SoftwareLimitSwitch
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(Degrees.of(0))
        ;

        profileConsts.update(motorConfig.MotionMagic, ElevatorConstants.sprocketRadius);
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
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.encoder.updateFrom(cancoder);
        inputs.leftMotor.updateFrom(leftMotor);
        inputs.rightMotor.updateFrom(rightMotor);

        if (profileConsts.hasChanged(hashCode())) {
            var config = new MotionMagicConfigs();
            leftMotor.getConfigurator().refresh(config);
            profileConsts.update(config, ElevatorConstants.sprocketRadius);
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

    @Override
    public void setVoltage(Measure<VoltageUnit> voltage) {
        leftMotor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void setLength(Measure<DistanceUnit> length) {
        leftMotor.setControl(positionRequest.withPosition(Radians.of(length.div(ElevatorConstants.sprocketRadius).baseUnitMagnitude() / 3)));
    }
}
