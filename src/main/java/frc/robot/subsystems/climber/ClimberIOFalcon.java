package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.constants.HardwareDevices;
import frc.util.loggerUtil.tunables.LoggedTunableAngularProfile;
import frc.util.loggerUtil.tunables.LoggedTunableFF;
import frc.util.loggerUtil.tunables.LoggedTunablePID;

public class ClimberIOFalcon implements ClimberIO {
    protected final TalonFX motor = HardwareDevices.climberMotorID.talonFX();
    protected final Servo servo = new Servo(HardwareDevices.climberServoPort);

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private static final LoggedTunableAngularProfile profileConsts = new LoggedTunableAngularProfile(
        "Climber/Profile",
        RotationsPerSecond.of(3),
        RotationsPerSecondPerSecond.of(6)
    );
    private static final LoggedTunableFF ffConsts = new LoggedTunableFF(
        "Climber/FF",
        0,
        0,
        0,
        0
    );
    private static final LoggedTunablePID pidConsts = new LoggedTunablePID(
        "Climber/PID",
        0,
        0,
        0
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
            .withReverseLimitAutosetPositionValue(Degrees.of(0))
        ;

        motorConfig.Feedback
            .withSensorToMechanismRatio(-1.0/ClimberConstants.sensorToMechanismRatio.ratio())
        ;

        profileConsts.update(motorConfig.MotionMagic);
        ffConsts.update(motorConfig.Slot0);
        pidConsts.update(motorConfig.Slot0);

        profileConsts.hasChanged(hashCode());
        ffConsts.hasChanged(hashCode());
        pidConsts.hasChanged(hashCode());

        motor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
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
    public void setRatchetServoAngle(Measure<AngleUnit> angle){
        servo.setAngle(angle.in(Degrees));
    }

    @Override
    public void setAngle(Measure<AngleUnit> angle){
        motor.setControl(positionRequest.withPosition(angle.in(Rotations)));
    }
}
