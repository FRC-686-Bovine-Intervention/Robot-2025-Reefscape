package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;
import frc.robot.subsystems.drive.OdometryThread.DoubleBuffer;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.faults.DeviceFaults;
import frc.util.faults.DeviceFaults.FaultType;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor.EncodedMotorStatusSignalCache;

public class ModuleIOFalcon550 implements ModuleIO {
    protected final TalonFX driveMotor;
    protected final SparkMax azimuthMotor;
    protected final AbsoluteEncoder azimuthAbsoluteEncoder;

    private final EncodedMotorStatusSignalCache driveMotorStatusSignalCache;

    private final DoubleBuffer drivePositionBuffer;
    private final DoubleBuffer azimuthPositionBuffer;

    private final VoltageOut driveVolts = new VoltageOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    protected final PIDController azimuthPID = new PIDController(0, 0, 0);

    public ModuleIOFalcon550(ModuleConstants config) {
        this.driveMotor = config.driveMotorID.talonFX();
        this.azimuthMotor = config.azimuthMotorID.sparkMax(MotorType.kBrushless);
        this.azimuthAbsoluteEncoder = this.azimuthMotor.getAbsoluteEncoder();

        var driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput
            .withInverted(config.driveInverted)
            .withNeutralMode(NeutralModeValue.Coast)
        ;
        driveConfig.ClosedLoopRamps
            .withVoltageClosedLoopRampPeriod(Seconds.of(0.075))
        ;
        // driveConfig.OpenLoopRamps
        //     .withVoltageOpenLoopRampPeriod(Seconds.of(0.1875))
        // ;
        driveConfig.CurrentLimits
            // .withSupplyCurrentLimit(Amps.of(70))
            // .withSupplyCurrentLowerLimit(Amps.of(70))
            // .withSupplyCurrentLowerTime(Seconds.of(0))
            .withSupplyCurrentLimitEnable(true)
            // .withStatorCurrentLimit(Amps.of(80))
            // .withStatorCurrentLimitEnable(true)
        ;
        
        this.driveMotor.getConfigurator().apply(driveConfig);

        var azimuthConfig = new SparkMaxConfig();
        azimuthConfig
            .idleMode(IdleMode.kCoast)
            .inverted(false)
            .smartCurrentLimit(40)
        ;
        azimuthConfig.absoluteEncoder
            .zeroOffset(config.encoderZeroOffset.in(Rotations))
            .inverted(true)
        ;
        azimuthConfig.signals.absoluteEncoderPositionPeriodMs((int) DriveConstants.odometryLoopFrequency.asPeriod().in(Milliseconds));

        this.azimuthMotor.configure(azimuthConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.azimuthPID.enableContinuousInput(
            0,
            1
        );

        this.driveMotorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.driveMotor);

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.driveMotorStatusSignalCache.encoder().getStatusSignals());
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(2), this.driveMotorStatusSignalCache.motor().getStatusSignals());
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getFaultStatusSignals(this.driveMotor));
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getStickyFaultStatusSignals(this.driveMotor));
        BaseStatusSignal.setUpdateFrequencyForAll(DriveConstants.odometryLoopFrequency, this.driveMotorStatusSignalCache.encoder().position());
        this.driveMotor.optimizeBusUtilization();

        this.drivePositionBuffer = OdometryThread.getInstance().registerPhoenixDoubleSignal(this.driveMotorStatusSignalCache.encoder().position(), Units::rotationsToRadians);
        this.azimuthPositionBuffer = OdometryThread.getInstance().registerGenericDoubleSignal(this.azimuthAbsoluteEncoder::getPosition, Units::rotationsToRadians);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            this.driveMotorStatusSignalCache.encoder().position(),
            this.driveMotorStatusSignalCache.encoder().velocity(),
            this.driveMotorStatusSignalCache.motor().appliedVoltage(),
            this.driveMotorStatusSignalCache.motor().statorCurrent(),
            this.driveMotorStatusSignalCache.motor().deviceTemperature()
        );
        inputs.driveMotorConnected = BaseStatusSignal.isAllGood(
            this.driveMotorStatusSignalCache.encoder().position(),
            this.driveMotorStatusSignalCache.encoder().velocity(),
            this.driveMotorStatusSignalCache.motor().appliedVoltage(),
            this.driveMotorStatusSignalCache.motor().statorCurrent(),
            this.driveMotorStatusSignalCache.motor().deviceTemperature()
        );
        inputs.azimuthMotorConnected = true;
        inputs.azimuthEncoderConnected = true;
        inputs.driveMotor.updateFrom(this.driveMotorStatusSignalCache);
        inputs.azimuthMotor.updateFrom(this.azimuthMotor);
        inputs.azimuthEncoder.updateFrom(this.azimuthAbsoluteEncoder);

        inputs.odometryDriveRads = this.drivePositionBuffer.popAll();
        inputs.odometryAzimuthRads = this.azimuthPositionBuffer.popAll();
        // inputs.driveMotorFaults.updateFrom(this.driveMotor);
        // inputs.azimuthMotorFaults.updateFrom(this.azimuthMotor);
    }

    @Override
    public void setDriveVoltage(Measure<VoltageUnit> volts) {
        this.driveMotor.setControl(this.driveVolts.withOutput(volts.in(Volts)));
    }
    @Override
    public void setDriveVelocity(Measure<AngularVelocityUnit> velocity, Measure<AngularAccelerationUnit> acceleration, Measure<VoltageUnit> feedforward, boolean overrideWithBrakeMode) {
        this.driveMotor.setControl(this.driveVelocity
            .withVelocity(velocity.in(RotationsPerSecond))
            .withAcceleration(acceleration.in(RotationsPerSecondPerSecond))
            .withFeedForward(feedforward.in(Volts))
            .withOverrideBrakeDurNeutral(overrideWithBrakeMode)
        );
    }

    protected void setAzimuthVolts(double volts) {
        this.azimuthMotor.setVoltage(volts);
    }
    @Override
    public void setAzimuthVoltage(Measure<VoltageUnit> volts) {
        this.setAzimuthVolts(volts.in(Volts));
    }
    @Override
    public void setAzimuthAngle(Measure<AngleUnit> angle) {
        this.setAzimuthVolts(
            this.azimuthPID.calculate(
                this.azimuthAbsoluteEncoder.getPosition(),
                angle.in(Rotations)
            )
        );
    }
    
    @Override
    public void stopDrive(Optional<NeutralMode> neutralMode) {
        this.driveMotor.setControl(neutralMode.map(NeutralMode::getPhoenix6ControlRequest).orElseGet(NeutralOut::new));
    }
    @Override
    public void stopAzimuth(Optional<NeutralMode> neutralMode) {
        //TODO Reimplement module turn brake mode
        // turnMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
        this.azimuthMotor.stopMotor();
    }

    @Override
    public void configDrivePID(PIDConstants pidConstants) {
        var config = new Slot0Configs();
        this.driveMotor.getConfigurator().refresh(config);
        pidConstants.update(config);
        this.driveMotor.getConfigurator().apply(config);
    }
    @Override
    public void configAzimuthPID(PIDConstants pidConstants) {
        pidConstants.update(this.azimuthPID);
    }

    @Override
    public void clearDriveStickyFaults(long bitmask) {
        if (bitmask == DeviceFaults.noneMask) {return;}
        if (bitmask == DeviceFaults.allMask) {
            this.driveMotor.clearStickyFaults();
        } else {
            for (var faultType : FaultType.possibleTalonFXFaults) {
                if (faultType.isPartOf(bitmask)) {
                    faultType.clearStickyFaultOn(this.driveMotor);
                }
            }
        }
    }
    @Override
    public void clearAzimuthStickyFaults(long bitmask) {
        if (bitmask == DeviceFaults.noneMask) {return;}
        this.azimuthMotor.clearFaults();
    }
}
