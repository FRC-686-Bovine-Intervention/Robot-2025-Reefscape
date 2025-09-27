package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Algae;
import frc.robot.constants.FieldConstants.Coral;
import frc.robot.constants.FieldConstants.Reef.BranchLevel;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.pivot.PivotConstants;

public class SuperstructureConstants {
    public static final Transform2d coralIntakeForwardTransform = new Transform2d(
        new Translation2d(
            Inches.of(12),
            Inches.of(-2)
        ),
        Rotation2d.kZero
    ).inverse();

    public static final Transform2d wristAxisToCoralTip = new Transform2d(
        new Translation2d(
            Inches.of(16.75),
            Inches.zero()
        ),
        Rotation2d.kZero
    );
    public static final Transform2d coralTipToWristAxis = wristAxisToCoralTip.inverse();

    public static final Transform2d wristAxisToAlgaeCenter = new Transform2d(
        new Translation2d(
            Inches.of(7.25).plus(Algae.radius),
            Inches.zero()
        ),
        Rotation2d.kZero
    );
    public static final Transform2d algaeCenterToWristAxis = wristAxisToAlgaeCenter.inverse();


    // Coral Intaking
    public static final SuperstructureState coralStationForwardState = SuperstructureState.fromWristAxisRobotSpace(
        new Pose2d(
            new Translation2d(
                RobotConstants.centerToFrontBumper.minus(Inches.of(3)),//.plus(Coral.radius.times(2)),
                FieldConstants.CoralStation.chuteBottomHeight.plus(Coral.radius.times(Math.cos(FieldConstants.CoralStation.chuteAngle.in(Radians)))).plus(Inches.of(-2.75))
            ),
            new Rotation2d(FieldConstants.CoralStation.chuteAngle)
        )
        .transformBy(SuperstructureConstants.coralIntakeForwardTransform)
    );
    public static final SuperstructureState coralStationBackwardState = SuperstructureState.fromParts(
        Degrees.of(85),
        ElevatorConstants.minLengthPhysical,
        Degrees.of(148)
    );

    // Algae Intaking
    public static final SuperstructureState highAlgaeState = SuperstructureState.fromAlgaeCenterRobotSpace(
        new Pose2d(
            new Translation2d(
                RobotConstants.centerToFrontBumper.plus(FieldConstants.Reef.minimumReefRadius).minus(FieldConstants.Reef.StagedAlgaeLevel.High.radiusFromReefCenter).plus(Inches.of(0)),
                FieldConstants.Reef.StagedAlgaeLevel.High.height.plus(Inches.of(3))
            ),
            Rotation2d.fromDegrees(-15)
        )
    );
    public static final SuperstructureState lowAlgaeState = SuperstructureState.fromAlgaeCenterRobotSpace(
        new Pose2d(
            new Translation2d(
                RobotConstants.centerToFrontBumper.plus(FieldConstants.Reef.minimumReefRadius).minus(FieldConstants.Reef.StagedAlgaeLevel.Low.radiusFromReefCenter).plus(Inches.of(0)),
                FieldConstants.Reef.StagedAlgaeLevel.Low.height.plus(Inches.of(3))
            ),
            Rotation2d.fromDegrees(-15)
        )
    );
    public static final SuperstructureState groundAlgaeState = SuperstructureState.fromParts(
        PivotConstants.minAngle,
        ElevatorConstants.minLengthPhysical,
        Degrees.of(-35)
    );

    // Reef Scoring
    public static final SuperstructureState l4State = SuperstructureState.fromCoralTipRobotSpace(
        new Pose2d(
            new Translation2d(
                RobotConstants.centerToFrontBumper.plus(Inches.of(2)),
                FieldConstants.Reef.BranchLevel.Level4.branchTipHeight.plus(Inches.of(4))
            ),
            Rotation2d.fromDegrees(-30)
        )
    );
    public static final SuperstructureState l4PreState = SuperstructureState.newConstrained(
        l4State.getPivotAngleRads(),
        l4State.getElevatorLengthMeters(),
        0
    );
    public static final SuperstructureState l3State = SuperstructureState.fromCoralTipRobotSpace(
        new Pose2d(
            new Translation2d(
                RobotConstants.centerToFrontBumper.plus(Inches.of(2)),
                FieldConstants.Reef.BranchLevel.Level3.branchTipHeight.plus(Inches.of(4))
            ),
            Rotation2d.fromDegrees(-15)
        )
    );
    public static final SuperstructureState l3l4TransferState = SuperstructureState.newConstrained(
        l4State.getPivotAngleRads(),
        l3State.getElevatorLengthMeters(),
        0
    );
    public static final SuperstructureState l2State = SuperstructureState.fromCoralTipRobotSpace(
        new Pose2d(
            new Translation2d(
                RobotConstants.centerToFrontBumper.plus(Inches.of(2)),
                FieldConstants.Reef.BranchLevel.Level2.branchTipHeight.plus(Inches.of(4))
            ),
            Rotation2d.fromDegrees(-15)
        )
    );
    public static final SuperstructureState l2l4TransferState = SuperstructureState.newConstrained(
        l4State.getPivotAngleRads(),
        l2State.getElevatorLengthMeters(),
        0
    );
    public static final SuperstructureState l1State = SuperstructureState.fromParts(
        PivotConstants.minAngle,
        ElevatorConstants.minLengthPhysical,
        Degrees.of(25)
    );
    public static SuperstructureState getStateForBranchLevel(BranchLevel branchLevel) {
        return switch (branchLevel) {
            case Level2 -> l2State;
            case Level3 -> l3State;
            case Level4 -> l4State;
        };
    }
    public static SuperstructureState getStateForBranchLevel(Optional<BranchLevel> branchLevel) {
        if (branchLevel.isEmpty()) {
            return l1State;
        }
        return getStateForBranchLevel(branchLevel.get());
    }

    // Algae Scoring
    public static final SuperstructureState processorState = SuperstructureState.fromParts(
        PivotConstants.minAngle,
        ElevatorConstants.minLengthPhysical,
        Degrees.of(10)
    );
    public static final SuperstructureState netForwardState = SuperstructureState.fromParts(
        Degrees.of(90),
        ElevatorConstants.maxLengthSoftware,
        Degrees.of(60)
    );
    public static final SuperstructureState netBackwardState = SuperstructureState.fromParts(
        Degrees.of(90),
        ElevatorConstants.maxLengthSoftware,
        Degrees.of(120)
    );
    public static final SuperstructureState netForwardPreState = SuperstructureState.fromParts(
        Degrees.of(90),
        ElevatorConstants.maxLengthSoftware,
        Degrees.of(120)
    );
    public static final SuperstructureState netBackwardPreState = SuperstructureState.fromParts(
        Degrees.of(90),
        ElevatorConstants.maxLengthSoftware,
        Degrees.of(60)
    );
    public static final SuperstructureState highAlgaeHoldState = SuperstructureState.fromParts(
        Degrees.of(90),
        Inches.of(12),
        Degrees.of(90)
    );
    public static final SuperstructureState lowAlgaeHoldState = SuperstructureState.fromParts(
        Degrees.of(90),
        Inches.of(12),
        Degrees.of(90)
    );

    public static final SuperstructureState idleState = SuperstructureState.fromParts(
        Radians.of(l4State.getPivotAngleRads()),
        ElevatorConstants.minLengthPhysical,
        Degrees.of(90)
    );
    public static final SuperstructureState defenseState = SuperstructureState.fromParts(
        PivotConstants.minAngle,
        ElevatorConstants.minLengthPhysical,
        Degrees.of(110)
    );
    public static final SuperstructureState prepareClimbingState = SuperstructureState.fromParts(
        Degrees.of(90),
        ElevatorConstants.minLengthPhysical,
        Degrees.of(-10)
    );
    public static final SuperstructureState climbingState = SuperstructureState.fromParts(
        Degrees.of(105),
        ElevatorConstants.minLengthPhysical,
        Degrees.of(180)
    );
    public static final SuperstructureState selfRightingState = SuperstructureState.fromParts(
        Degrees.of(112),
        ElevatorConstants.minLengthPhysical,
        Degrees.of(180)
    );
}
