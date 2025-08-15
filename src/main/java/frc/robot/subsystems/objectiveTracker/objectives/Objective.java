package frc.robot.subsystems.objectiveTracker.objectives;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.superstructure.Superstructure.Direction;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.util.flipping.AllianceFlipped;

public interface Objective {
    public static enum ObjectiveType {
        IntakeCoral(false),
        IntakeAlgae(true),
        ScoreCoral(true),
        ScoreAlgae(false),
        Climb(false),
        ;
        public final boolean isReefObjective;
        ObjectiveType(boolean isReefObjective) {
            this.isReefObjective = isReefObjective;
        }
    }

    public AllianceFlipped<Pose2d> getTargetPose();
    public SuperstructureState getTargetState();
    public Direction getTargetDirection();
    public ObjectiveType getObjectiveType();
}
