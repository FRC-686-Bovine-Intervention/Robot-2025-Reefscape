package frc.robot.subsystems.objectiveTracker.objectives;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.superstructure.Superstructure.Direction;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.util.flipping.AllianceFlipped;

public class ClimbObjective implements Objective {
    private final AllianceFlipped<Pose2d> targetRobotPose;

    public ClimbObjective(AllianceFlipped<Pose2d> targetRobotPose) {
        this.targetRobotPose = targetRobotPose;
    }

    @Override
    public AllianceFlipped<Pose2d> getTargetPose() {
        return this.targetRobotPose;
    }

    @Override
    public SuperstructureState getTargetState() {
        return SuperstructureConstants.prepareClimbingState;
    }

    @Override
    public Direction getTargetDirection() {
        return Direction.Forward;
    }

    @Override
    public ObjectiveType getObjectiveType() {
        return ObjectiveType.Climb;
    }
}
