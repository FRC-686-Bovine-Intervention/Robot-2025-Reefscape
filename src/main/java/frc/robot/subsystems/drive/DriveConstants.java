package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;
import frc.util.Environment;
import frc.util.geometry.GeomUtil;
import frc.util.hardwareID.can.CANDevice;
import frc.util.mechanismUtil.GearRatio;
import frc.util.mechanismUtil.LinearRelation;

public final class DriveConstants {
    public static final double odometryLoopFrequencyHz = RobotConstants.rioUpdateFrequencyHz;
    public static final Frequency odometryLoopFrequency = Hertz.of(odometryLoopFrequencyHz);

    /**Distance between the front and back wheels*/
    public static final Distance trackWidthX = Inches.of(25.5);
    /**Distance between the left and right wheels*/
    public static final Distance trackWidthY = Inches.of(25.5);

    public static class ModuleConstants {
        public final String name;
        public final CANDevice driveMotorID;
        public final CANDevice azimuthMotorID;
        public final InvertedValue driveInverted;
        public final Rotation2d moduleForwardDirection;
        public final Angle encoderZeroOffset;
        public final Translation2d moduleTranslation;
        public final Vector<N2> positiveRotVec;
        ModuleConstants(String name, CANDevice driveMotorID, CANDevice turnMotorID, InvertedValue driveInverted, Rotation2d moduleForwardDirection, Angle encoderZeroOffset, Translation2d moduleTranslation) {
            this.name = name;
            this.driveMotorID = driveMotorID;
            this.azimuthMotorID = turnMotorID;
            this.driveInverted = driveInverted;
            this.moduleForwardDirection = moduleForwardDirection;
            this.encoderZeroOffset = encoderZeroOffset;
            this.moduleTranslation = moduleTranslation;
            this.positiveRotVec = GeomUtil.vectorFromRotation(this.moduleTranslation.getAngle().plus(Rotation2d.fromDegrees(90)));
        }
    }

    public static final ModuleConstants[] moduleConstants = {
        new ModuleConstants(
            "Front Left",
            HardwareDevices.frontLeftDriveMotorID, HardwareDevices.frontLeftTurnMotorID,
            InvertedValue.CounterClockwise_Positive,
            Rotation2d.kCCW_90deg,
            Rotations.of(0.7326064),
            new Translation2d(
                trackWidthX.div(+2),
                trackWidthY.div(+2)
            )
        ),
        new ModuleConstants(
            "Front Right",
            HardwareDevices.frontRightDriveMotorID, HardwareDevices.frontRightTurnMotorID,
            InvertedValue.CounterClockwise_Positive,
            Rotation2d.kZero,
            Rotations.of(0.7325602),
            new Translation2d(
                trackWidthX.div(+2),
                trackWidthY.div(-2)
            )
        ),
        new ModuleConstants(
            "Back Left",
            HardwareDevices.backLeftDriveMotorID, HardwareDevices.backLeftTurnMotorID,
            InvertedValue.CounterClockwise_Positive,
            Rotation2d.k180deg,
            Rotations.of(0.6867877),
            new Translation2d(
                trackWidthX.div(-2),
                trackWidthY.div(+2)
            )
        ),
        new ModuleConstants(
            "Back Right",
            HardwareDevices.backRightDriveMotorID, HardwareDevices.backRightTurnMotorID,
            InvertedValue.CounterClockwise_Positive,
            Rotation2d.kCW_90deg,
            Rotations.of(0.8682748),
            new Translation2d(
                trackWidthX.div(-2),
                trackWidthY.div(-2)
            )
        ),
    };
    public static final Translation2d[] moduleTranslations = Arrays.stream(moduleConstants).map((a) -> a.moduleTranslation).toArray(Translation2d[]::new);
    public static final Distance driveBaseRadius = Meters.of(Arrays.stream(moduleTranslations).mapToDouble((t) -> t.getNorm()).max().orElse(0.5));

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);

    public static final LinearRelation wheel = LinearRelation.wheelRadius(Inches.of(1.53));

    public static final GearRatio driveMotorToWheelRatio = new GearRatio()
        .gear(14).gear(22).axle()
        .gear(15).gear(45).axle()
    ;
    public static final GearRatio azimuthMotorToEncoderRatio = new GearRatio()
        .gear(15).gear(32).axle()
        .gear(10).gear(60).axle()
    ;
    public static final GearRatio azimuthEncoderToCarriageRatio = new GearRatio();
    public static final GearRatio azimuthMotorToCarriageRatio = azimuthMotorToEncoderRatio.then(azimuthEncoderToCarriageRatio);

    public static final LinearVelocity maxModuleSpeed = wheel.angularVelocityToLinearVelocity(driveMotorToWheelRatio.applyUnsigned(RadiansPerSecond.of(DCMotor.getFalcon500(1).freeSpeedRadPerSec)));

    public static final LinearVelocity maxDriveSpeed = MetersPerSecond.of(6);
    /**Tangential speed (m/s) = radial speed (rad/s) * radius (m)*/
    public static final AngularVelocity maxTurnRate = RadiansPerSecond.of(maxDriveSpeed.in(MetersPerSecond) / driveBaseRadius.in(Meters));
    public static final DoubleSupplier maxDriveSpeedEnvCoef = Environment.switchVar(
        () -> 1.0,
        new LoggedNetworkNumber("Demo Constraints/Max Translational Percentage", 0.25)::get
    );
    public static final DoubleSupplier maxTurnRateEnvCoef = Environment.switchVar(
        () -> 1.0,
        new LoggedNetworkNumber("Demo Constraints/Max Rotational Percentage", 0.5)::get
    );

    public static final LinearVelocity maxAdjustmentSpeed = InchesPerSecond.of(12);

    public static final RobotConfig robotConfig = new RobotConfig(
        RobotConstants.robotWeight,
        RobotConstants.robotMOI,
        new com.pathplanner.lib.config.ModuleConfig(
            DriveConstants.wheel.effectiveRadius(),
            DriveConstants.maxDriveSpeed,
            1.0,
            DCMotor.getFalcon500(1).withReduction(driveMotorToWheelRatio.reductionUnsigned()),
            Amps.of(80),
            1
        ),
        DriveConstants.moduleTranslations
    );
}