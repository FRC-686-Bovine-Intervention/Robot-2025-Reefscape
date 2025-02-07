package frc.robot.auto;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class ScoreCoral extends AutoRoutine {
    public ScoreCoral(RobotContainer robot) {
        super("ScoreCoral", List.of());
    }

    @Override
    public Command generateCommand() {
        return Commands.none();
    }
    
}
