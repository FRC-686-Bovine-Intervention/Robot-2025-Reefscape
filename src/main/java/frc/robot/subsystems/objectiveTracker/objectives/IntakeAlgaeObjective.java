package frc.robot.subsystems.objectiveTracker.objectives;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.FieldConstants.Reef.StagedAlgaeConcept;
import frc.robot.subsystems.superstructure.Superstructure.Direction;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.util.flipping.AllianceFlipped;

public class IntakeAlgaeObjective implements Objective {
    private final AllianceFlipped<Pose2d> targetRobotPose;
    private final StagedAlgaeConcept targetAlgae;

    public IntakeAlgaeObjective(AllianceFlipped<Pose2d> targetRobotPose, StagedAlgaeConcept targetAlgae) {
        this.targetRobotPose = targetRobotPose;
        this.targetAlgae = targetAlgae;
    }

    @Override
    public AllianceFlipped<Pose2d> getTargetPose() {
        return this.targetRobotPose;
    }

    @Override
    public SuperstructureState getTargetState() {
        return switch (this.targetAlgae.level) {
            case High -> SuperstructureConstants.highAlgaeState;
            case Low -> SuperstructureConstants.lowAlgaeState;
        };
    }

    @Override
    public Direction getTargetDirection() {
        return Direction.Forward;
    }

    @Override
    public ObjectiveType getObjectiveType() {
        return ObjectiveType.IntakeAlgae;
    }

    public StagedAlgaeConcept getTargetAlgae() {
        return this.targetAlgae;
    }
}
