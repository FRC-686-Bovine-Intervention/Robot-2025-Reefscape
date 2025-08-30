package frc.robot.subsystems.objectiveTracker.objectives;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.superstructure.Superstructure.Direction;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.util.flipping.AllianceFlipped;

public class ScoreProcessorObjective implements Objective {
    private final AllianceFlipped<Pose2d> targetRobotPose;

    public ScoreProcessorObjective(AllianceFlipped<Pose2d> targetRobotPose) {
        this.targetRobotPose = targetRobotPose;
    }

    @Override
    public AllianceFlipped<Pose2d> getTargetPose() {
        return this.targetRobotPose;
    }

    @Override
    public SuperstructureState getTargetState() {
        return SuperstructureConstants.processorState;
    }

    @Override
    public Direction getTargetDirection() {
        return Direction.Forward;
    }

    @Override
    public ObjectiveType getObjectiveType() {
        return ObjectiveType.ScoreProcessor;
    }
}
