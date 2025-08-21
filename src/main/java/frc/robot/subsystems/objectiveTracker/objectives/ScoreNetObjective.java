package frc.robot.subsystems.objectiveTracker.objectives;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.superstructure.Superstructure.Direction;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.util.flipping.AllianceFlipped;

public class ScoreNetObjective implements Objective {
    private final AllianceFlipped<Pose2d> targetRobotPose;
    private final Direction direction;

    public ScoreNetObjective(AllianceFlipped<Pose2d> targetRobotPose, Direction direction) {
        this.targetRobotPose = targetRobotPose;
        this.direction = direction;
    }

    @Override
    public AllianceFlipped<Pose2d> getTargetPose() {
        return this.targetRobotPose;
    }

    @Override
    public SuperstructureState getTargetState() {
        return switch (this.getTargetDirection()) {
            case Forward -> SuperstructureConstants.netForwardState;
            case Backward -> SuperstructureConstants.netBackwardState;
        };
    }

    @Override
    public Direction getTargetDirection() {
        return this.direction;
    }

    @Override
    public ObjectiveType getObjectiveType() {
        return ObjectiveType.ScoreNet;
    }
}
