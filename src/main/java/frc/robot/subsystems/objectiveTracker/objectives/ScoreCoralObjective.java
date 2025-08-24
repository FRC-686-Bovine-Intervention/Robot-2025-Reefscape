package frc.robot.subsystems.objectiveTracker.objectives;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.FieldConstants.Reef.BranchConcept;
import frc.robot.subsystems.superstructure.Superstructure.Direction;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.util.flipping.AllianceFlipped;

public class ScoreCoralObjective implements Objective {
    private final AllianceFlipped<Pose2d> targetRobotPose;
    private final Optional<BranchConcept> targetBranch;
    private final Direction direction;

    public ScoreCoralObjective(AllianceFlipped<Pose2d> targetRobotPose, Optional<BranchConcept> targetBranch, Direction direction) {
        this.targetRobotPose = targetRobotPose;
        this.targetBranch = targetBranch;
        this.direction = direction;
    }

    @Override
    public AllianceFlipped<Pose2d> getTargetPose() {
        return this.targetRobotPose;
    }

    @Override
    public SuperstructureState getTargetState() {
        return SuperstructureConstants.getStateForBranchLevel(this.targetBranch.map((branch) -> branch.level));
    }

    @Override
    public Direction getTargetDirection() {
        return this.direction;
    }

    @Override
    public ObjectiveType getObjectiveType() {
        return ObjectiveType.ScoreCoral;
    }

    public Optional<BranchConcept> getTargetBranch() {
        return this.targetBranch;
    }
}
