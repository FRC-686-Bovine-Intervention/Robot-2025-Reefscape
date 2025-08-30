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
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.Module;
import frc.util.geometry.GeomUtil;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class WheelRadiusCalibration extends Command {
    private final Drive drive;
    private final MutAngle prevYaw = Radians.mutable(0);
    private final MutAngle totalYaw = Radians.mutable(0);
    private final Timer totalTimer = new Timer();
    private final Measure<VoltageUnit> maxVoltage;
    private final Measure<VelocityUnit<VoltageUnit>> voltageRampRate;
    private double[] initialPositionRads = new double[0];

    public static final LoggedTunable<Velocity<VoltageUnit>> VOLTAGE_RAMP_RATE = LoggedTunable.from("Drive/Wheel Calibration/Voltage Ramp Rate", Volts.per(Second)::of, 2);
    public static final LoggedTunable<Voltage> MAX_VOLTAGE = LoggedTunable.from("Drive/Wheel Calibration/Max Voltage", Volts::of, 6);

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
        this.initialPositionRads = Arrays.stream(this.drive.modules).mapToDouble(Module::getWheelAngularPositionRads).toArray();
    }

    @Override
    public void execute() {
        var yaw = RobotState.getInstance().getEstimatedGlobalPose().getRotation().getMeasure();
        var yawDiff = yaw.minus(this.prevYaw).in(Radians);
        var wrappedDiff = MathUtil.angleModulus(yawDiff);
        this.totalYaw.mut_acc(wrappedDiff);

        this.prevYaw.mut_replace(yaw);

        var averageWheelRadians = IntStream.range(0, drive.modules.length)
            .mapToDouble((i) -> drive.modules[i].getWheelAngularPositionRads() - this.initialPositionRads[i])
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
