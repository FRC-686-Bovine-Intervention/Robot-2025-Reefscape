package frc.robot.subsystems.drive.commands;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.util.geometry.GeomUtil;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;

public class MOICharacterization extends Command {
    private final Drive drive;
    private final DCMotor driveMotor;
    private final Measure<VoltageUnit> voltage;

    private final MutAngularVelocity prevVelo = RadiansPerSecond.mutable(0);
    private final MutTime prevTime = Seconds.mutable(0);

    public static final LoggedTunableMeasure<VoltageUnit> VOLTAGE = new LoggedTunableMeasure<>("Drive/MOI Characterization/Voltage", Volts.of(4));

    public MOICharacterization(Drive drive, DCMotor driveMotor, Measure<VoltageUnit> maxVoltage) {
        this.drive = drive;
        addRequirements(this.drive.translationSubsystem, this.drive.rotationalSubsystem);
        setName("MOI Characterization");
        this.driveMotor = driveMotor;
        this.voltage = maxVoltage;
    }

    @Override
    public void initialize() {
        prevVelo.mut_replace(drive.getYawVelocity());
        prevTime.mut_replace(Timer.getTimestamp(), Seconds);
    }

    @Override
    public void execute() {
        var gyroVelo = drive.getYawVelocity();
        var time = Seconds.of(Timer.getTimestamp());

        var deltaVelocityRPS = Math.abs(gyroVelo.minus(prevVelo).in(RadiansPerSecond));
        var deltaTimeSec = time.minus(prevTime).in(Seconds);

        var gyroAccelRPSS = deltaVelocityRPS / deltaTimeSec;

        var totalChassisTorqueNM = Arrays.stream(drive.modules)
            .mapToDouble((module) -> Math.abs(module.getDriveCurrent().in(Amps)))
            .map((moduleCurrentAmps) -> driveMotor.getTorque(moduleCurrentAmps))
            .map((wheelTorqueNM) -> wheelTorqueNM / DriveConstants.wheel.effectiveRadius().in(Meters))
            .map((wheelForceN) -> wheelForceN * DriveConstants.driveBaseRadius.in(Meters))
            .reduce(0, (acc, ele) -> acc + ele)
        ;

        var moiKgMM = totalChassisTorqueNM / gyroAccelRPSS;

        Logger.recordOutput("Drive/MOI Characterization/Total Chassis Torque", totalChassisTorqueNM);
        Logger.recordOutput("Drive/MOI Characterization/Gyro Accel", gyroAccelRPSS);
        Logger.recordOutput("Drive/MOI Characterization/MOI", moiKgMM);

        Arrays.stream(drive.modules).forEach((module) -> module.runVoltage(voltage, GeomUtil.rotationFromVector(module.config.positiveRotVec)));

        prevVelo.mut_replace(gyroVelo);
        prevTime.mut_replace(time);
    }

    @Override
    public void end(boolean interrupted) {
        drive.runRobotSpeeds(new ChassisSpeeds());
    }
}
