package frc.robot.subsystems.objectiveTracker.objectives;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.superstructure.Superstructure.Direction;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.util.flipping.AllianceFlipped;

public class IntakeCoralObjective implements Objective {
    private final AllianceFlipped<Pose2d> targetRobotPose;
    private final Direction direction;

    public IntakeCoralObjective(AllianceFlipped<Pose2d> targetRobotPose, Direction direction) {
        this.targetRobotPose = targetRobotPose;
        this.direction = direction;
    }

    @Override
    public AllianceFlipped<Pose2d> getTargetPose() {
        return this.targetRobotPose;
    }

    @Override
    public SuperstructureState getTargetState() {
        return FieldConstants.CoralStation.intakePosition.get(this.getTargetDirection());
    }

    @Override
    public Direction getTargetDirection() {
        return this.direction;
    }

    @Override
    public ObjectiveType getObjectiveType() {
        return ObjectiveType.IntakeCoral;
    }
}
