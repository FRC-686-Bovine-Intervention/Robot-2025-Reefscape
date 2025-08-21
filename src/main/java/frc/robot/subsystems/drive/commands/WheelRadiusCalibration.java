package frc.robot.subsystems.drive.commands;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;
import java.util.stream.IntStream;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.Module;
import frc.util.geometry.GeomUtil;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;

public class WheelRadiusCalibration extends Command {
    private final Drive drive;
    private final MutAngle prevYaw = Radians.mutable(0);
    private final MutAngle totalYaw = Radians.mutable(0);
    private final Timer totalTimer = new Timer();
    private final Measure<VoltageUnit> maxVoltage;
    private final Measure<VelocityUnit<VoltageUnit>> voltageRampRate;
    private Angle[] initialPositions = new Angle[0];

    public static final LoggedTunableMeasure<VelocityUnit<VoltageUnit>> VOLTAGE_RAMP_RATE = new LoggedTunableMeasure<>("Drive/Wheel Calibration/Voltage Ramp Rate", Volts.per(Second).of(2));
    public static final LoggedTunableMeasure<VoltageUnit> MAX_VOLTAGE = new LoggedTunableMeasure<>("Drive/Wheel Calibration/Max Voltage", Volts.of(6));

    public WheelRadiusCalibration(Drive drive, Measure<VelocityUnit<VoltageUnit>> voltageRampRate, Measure<VoltageUnit> maxVoltage) {
        this.drive = drive;
        addRequirements(this.drive.translationSubsystem, this.drive.rotationalSubsystem);
        setName("Wheel Calibration");
        this.voltageRampRate = voltageRampRate;
        this.maxVoltage = maxVoltage;
    }

    @Override
    public void initialize() {
        this.totalTimer.restart();
        this.prevYaw.mut_replace(RobotState.getInstance().getEstimatedGlobalPose().getRotation().getMeasure());
        this.totalYaw.mut_replace(Radians.zero());
        initialPositions = Arrays.stream(this.drive.modules).map(Module::getWheelAngularPosition).map(Angle::copy).toArray(Angle[]::new);
    }

    @Override
    public void execute() {
        var yaw = RobotState.getInstance().getEstimatedGlobalPose().getRotation().getMeasure();
        var yawDiff = yaw.minus(this.prevYaw).in(Radians);
        var wrappedDiff = MathUtil.angleModulus(yawDiff);
        this.totalYaw.mut_acc(wrappedDiff);

        this.prevYaw.mut_replace(yaw);

        var averageWheelRadians = IntStream.range(0, drive.modules.length)
            .mapToDouble((i) -> drive.modules[i].getWheelAngularPosition().minus(initialPositions[i]).in(Radians))
            .average().orElse(0)
        ;

        var averageWheelRadius = DriveConstants.driveBaseRadius.times(totalYaw.in(Radians)).div(averageWheelRadians);

        Logger.recordOutput("Drive/Wheel Calibration/Total Yaw", totalYaw);
        Logger.recordOutput("Drive/Wheel Calibration/Average Wheel Radians", averageWheelRadians);
        Logger.recordOutput("Drive/Wheel Calibration/Expected Wheel Travel", DriveConstants.driveBaseRadius.times(totalYaw.in(Radians)));
        Logger.recordOutput("Drive/Wheel Calibration/Average Wheel Radius", averageWheelRadius);

        var volts = Math.min(voltageRampRate.in(Volts.per(Second)) * totalTimer.get(), maxVoltage.in(Volts));
        Arrays.stream(drive.modules).forEach((module) -> module.runVoltage(Volts.of(volts), GeomUtil.rotationFromVector(module.config.positiveRotVec)));
    }

    @Override
    public void end(boolean interrupted) {
        totalTimer.stop();
        drive.runRobotSpeeds(new ChassisSpeeds());
    }
}
