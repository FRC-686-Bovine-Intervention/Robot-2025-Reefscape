package frc.robot.auto;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;

public class DrivePastLine extends AutoRoutine{
    
    private final Drive drive;
    public DrivePastLine(RobotContainer robot){
        super("DrivePastLine", List.of());
        this.drive = robot.drive;
    }


    @Override
    public Command generateCommand() {
        return Commands.run(() -> drive.runRobotSpeeds(null), drive.translationSubsystem, drive.rotationalSubsystem).raceWith(Commands.waitSeconds(0.5));
    }
}
