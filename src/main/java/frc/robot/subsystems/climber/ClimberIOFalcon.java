package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.util.loggerUtil.tunables.LoggedTunableAngularProfile;
import frc.util.loggerUtil.tunables.LoggedTunableFF;
import frc.util.loggerUtil.tunables.LoggedTunablePID;

public class ClimberIOFalcon implements ClimberIO {
    protected final TalonFX motor = HardwareDevices.climberMotorID.talonFX();
    protected final Servo servo = HardwareDevices.climberServoPort.servo();
    protected final DigitalInput sensor = HardwareDevices.climberSensor.input();

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
        4,
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

        BaseStatusSignal.setUpdateFrequencyForAll(
            RobotConstants.rioUpdateFrequency,
            motor.getRotorPosition(),
            motor.getRotorVelocity()
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
            RobotConstants.rioUpdateFrequency,
            motor.getPosition(),
            motor.getVelocity(),
            motor.getClosedLoopReference(),
            motor.getClosedLoopReferenceSlope(),
            motor.getClosedLoopError(),
            motor.getClosedLoopOutput()
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
            DriveConstants.odometryLoopFrequency.div(2),
            motor.getMotorVoltage(),
            motor.getStatorCurrent(),
            motor.getDeviceTemp()
        );
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.motor.updateFrom(motor);

        inputs.sensor = sensor.get() ^ ClimberConstants.climberSensorInverted;

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

        Logger.recordOutput("Climber/Motor/posiion", motor.getPosition().getValueAsDouble());
        Logger.recordOutput("Climber/Motor/veloctiy", motor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Climber/Motor/Profile/Position", motor.getClosedLoopReference().getValueAsDouble());
        Logger.recordOutput("Climber/Motor/Profile/Velocity", motor.getClosedLoopReferenceSlope().getValueAsDouble());
        Logger.recordOutput("Climber/Motor/PID error", motor.getClosedLoopError().getValueAsDouble());
        Logger.recordOutput("Climber/Motor/Out", motor.getClosedLoopOutput().getValueAsDouble());
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
