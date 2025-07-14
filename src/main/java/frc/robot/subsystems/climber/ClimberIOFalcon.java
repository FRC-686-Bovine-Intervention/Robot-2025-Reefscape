package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
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

    private final VoltageOut voltageRequest = new VoltageOut(0);

    private final MotionMagicVoltage nonClimbingPositionRequest = new MotionMagicVoltage(0)
        .withSlot(0)
    ;
    private final MotionMagicVoltage climbingPositionRequest = new MotionMagicVoltage(0)
        .withSlot(1)
        .withOverrideBrakeDurNeutral(true)
        .withLimitForwardMotion(true)
    ;

    private static final LoggedTunableAngularProfile profileConsts = new LoggedTunableAngularProfile(
        "Climber/Profile",
        RotationsPerSecond.of(6),
        RotationsPerSecondPerSecond.of(12)
    );
    private static final LoggedTunableFF nonClimbingFFConsts = new LoggedTunableFF(
        "Climber/Nonclimbing/FF",
        0,
        0,
        0,
        0
    );
    private static final LoggedTunablePID nonClimbingPIDConsts = new LoggedTunablePID(
        "Climber/Nonclimbing/PID",
        8,
        0,
        0
    );
    private static final LoggedTunableFF climbingFFConsts = new LoggedTunableFF(
        "Climber/Climbing/FF",
        0,
        0,
        0,
        0
    );
    private static final LoggedTunablePID climbingPIDConsts = new LoggedTunablePID(
        "Climber/Climbing/PID",
        16,
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
            .withReverseLimitAutosetPositionValue(ClimberConstants.climberMinimumAngle)
        ;

        motorConfig.Feedback
            .withSensorToMechanismRatio(ClimberConstants.sensorToMechanismRatio.reductionUnsigned())
        ;

        profileConsts.update(motorConfig.MotionMagic);
        nonClimbingFFConsts.update(motorConfig.Slot0);
        nonClimbingPIDConsts.update(motorConfig.Slot0);
        climbingFFConsts.update(motorConfig.Slot1);
        climbingPIDConsts.update(motorConfig.Slot1);

        profileConsts.hasChanged(hashCode());
        nonClimbingFFConsts.hasChanged(hashCode());
        nonClimbingPIDConsts.hasChanged(hashCode());
        climbingFFConsts.hasChanged(hashCode());
        climbingPIDConsts.hasChanged(hashCode());

        motor.getConfigurator().apply(motorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
            RobotConstants.rioUpdateFrequency,
            motor.getRotorPosition(),
            motor.getRotorVelocity()
        );
        // BaseStatusSignal.setUpdateFrequencyForAll(
        //     RobotConstants.rioUpdateFrequency,
        //     motor.getPosition(),
        //     motor.getVelocity(),
        //     motor.getClosedLoopReference(),
        //     motor.getClosedLoopReferenceSlope(),
        //     motor.getClosedLoopError(),
        //     motor.getClosedLoopOutput()
        // );
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

        voltageRequest.withLimitReverseMotion(inputs.sensor);
        nonClimbingPositionRequest.withLimitReverseMotion(inputs.sensor);
        climbingPositionRequest.withLimitReverseMotion(inputs.sensor);

        if (profileConsts.hasChanged(hashCode())) {
            var config = new MotionMagicConfigs();
            motor.getConfigurator().refresh(config);
            profileConsts.update(config);
            motor.getConfigurator().apply(config);
        }

        if (nonClimbingFFConsts.hasChanged(hashCode()) | nonClimbingPIDConsts.hasChanged(hashCode())) {
            var config = new Slot0Configs();
            motor.getConfigurator().refresh(config);
            nonClimbingFFConsts.update(config);
            nonClimbingPIDConsts.update(config);
            motor.getConfigurator().apply(config);
        }

        if (climbingFFConsts.hasChanged(hashCode()) | climbingPIDConsts.hasChanged(hashCode())) {
            var config = new Slot1Configs();
            motor.getConfigurator().refresh(config);
            climbingFFConsts.update(config);
            climbingPIDConsts.update(config);
            motor.getConfigurator().apply(config);
        }

        // Logger.recordOutput("Climber/Motor/posiion", motor.getPosition().getValueAsDouble());
        // Logger.recordOutput("Climber/Motor/veloctiy", motor.getVelocity().getValueAsDouble());
        // Logger.recordOutput("Climber/Motor/Profile/Position", motor.getClosedLoopReference().getValueAsDouble());
        // Logger.recordOutput("Climber/Motor/Profile/Velocity", motor.getClosedLoopReferenceSlope().getValueAsDouble());
        // Logger.recordOutput("Climber/Motor/PID error", motor.getClosedLoopError().getValueAsDouble());
        // Logger.recordOutput("Climber/Motor/Out", motor.getClosedLoopOutput().getValueAsDouble());
    }

    @Override
    public void setVoltage(Measure<VoltageUnit> voltage, boolean brakeMode) {
        motor.setControl(voltageRequest.withOutput(voltage.in(Volts)).withOverrideBrakeDurNeutral(brakeMode));
    }

    @Override
    public void setRatchetServoAngle(Measure<AngleUnit> angle) {
        servo.setAngle(angle.in(Degrees));
    }

    @Override
    public void setNonClimbingAngle(Measure<AngleUnit> angle) {
        motor.setControl(nonClimbingPositionRequest.withPosition(angle.in(Rotations)));
    }

    @Override
    public void setClimbingAngle(Measure<AngleUnit> angle) {
        motor.setControl(climbingPositionRequest.withPosition(angle.in(Rotations)));
    }
}
