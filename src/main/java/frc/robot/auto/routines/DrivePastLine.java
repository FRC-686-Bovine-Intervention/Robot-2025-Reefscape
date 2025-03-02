package frc.robot.auto.routines;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoRoutine;
import frc.robot.subsystems.drive.Drive;

public class DrivePastLine extends AutoRoutine{
    
    private final Drive drive;
    public DrivePastLine(RobotContainer robot){
        super("DrivePastLine", List.of());
        this.drive = robot.drive;
    }


    @Override
    public Command generateCommand() {
        return Commands.runEnd(
            () -> drive.runRobotSpeeds(
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    MetersPerSecond.of(3),
                    MetersPerSecond.zero(),
                    DegreesPerSecond.zero(),
                    Rotation2d.k180deg
                )),
            () -> drive.stop(),
            drive.translationSubsystem, drive.rotationalSubsystem
        ).raceWith(Commands.waitSeconds(0.5));
    }
}
