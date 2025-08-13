package frc.robot.subsystems.objectiveTracker.objectives;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.superstructure.Superstructure.Direction;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.util.flipping.AllianceFlipped;

public class ScoreAlgaeObjective implements Objective {
    private final AllianceFlipped<Pose2d> targetRobotPose;
    private final boolean isProcessor;
    private final Direction direction;

    public ScoreAlgaeObjective(AllianceFlipped<Pose2d> targetRobotPose, boolean isProcessor, Direction direction) {
        this.targetRobotPose = targetRobotPose;
        this.isProcessor = isProcessor;
        this.direction = direction;
    }

    @Override
    public AllianceFlipped<Pose2d> getTargetPose() {
        return this.targetRobotPose;
    }

    @Override
    public SuperstructureState getTargetState() {
        if (isProcessor) {
            return FieldConstants.Processor.superstructureState.get(this.getTargetDirection());
        } else {
            return FieldConstants.Barge.superstructureState.get(this.getTargetDirection());
        }
    }

    @Override
    public Direction getTargetDirection() {
        return this.direction;
    }

    @Override
    public ObjectiveType getObjectiveType() {
        if (isProcessor) {
            return ObjectiveType.ScoreProcessor;
        } else {
            return ObjectiveType.ScoreNet;
        }
    }

    public boolean isProcessor() {
        return this.isProcessor;
    }
}
