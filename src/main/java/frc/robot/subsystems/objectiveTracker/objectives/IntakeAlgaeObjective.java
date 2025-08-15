package frc.robot.subsystems.objectiveTracker.objectives;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.FieldConstants.Reef.StagedAlgaeConcept;
import frc.robot.subsystems.superstructure.Superstructure.Direction;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.util.flipping.AllianceFlipped;

public class IntakeAlgaeObjective implements Objective {
    private final AllianceFlipped<Pose2d> targetRobotPose;
    private final StagedAlgaeConcept targetAlgae;
    private final Direction direction;

    public IntakeAlgaeObjective(AllianceFlipped<Pose2d> targetRobotPose, StagedAlgaeConcept targetAlgae, Direction direction) {
        this.targetRobotPose = targetRobotPose;
        this.targetAlgae = targetAlgae;
        this.direction = direction;
    }

    @Override
    public AllianceFlipped<Pose2d> getTargetPose() {
        return this.targetRobotPose;
    }

    @Override
    public SuperstructureState getTargetState() {
        return this.targetAlgae.level.intakeSuperstructureStates.get(this.getTargetDirection());
    }

    @Override
    public Direction getTargetDirection() {
        return this.direction;
    }

    @Override
    public ObjectiveType getObjectiveType() {
        return ObjectiveType.IntakeAlgae;
    }

    public StagedAlgaeConcept getTargetAlgae() {
        return this.targetAlgae;
    }
}
