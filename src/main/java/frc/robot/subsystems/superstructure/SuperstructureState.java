package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.pivot.PivotConstants;
import frc.robot.subsystems.superstructure.wrist.WristConstants;

public class SuperstructureState {
    private double pivotAngleRads;
    private double elevatorLengthMeters;
    private double wristAngleRads;

    private SuperstructureState(double pivotAngleRads, double elevatorLengthMeters, double wristAngleRads) {
        this.pivotAngleRads = pivotAngleRads;
        this.elevatorLengthMeters = elevatorLengthMeters;
        this.wristAngleRads = wristAngleRads;
    }

    public static SuperstructureState newUnconstrained(double pivotAngleRads, double elevatorLengthMeters, double wristAngleRads) {
        return new SuperstructureState(pivotAngleRads, elevatorLengthMeters, wristAngleRads);
    }
    public static SuperstructureState newUnconstrained(Measure<AngleUnit> pivotAngle, Measure<DistanceUnit> elevatorLength, Measure<AngleUnit> wristAngle) {
        return SuperstructureState.newUnconstrained(pivotAngle.in(Radians), elevatorLength.in(Meters), wristAngle.in(Radians));
    }

    public static SuperstructureState newConstrained(double pivotAngleRads, double elevatorLengthMeters, double wristAngleRads) {
        return new SuperstructureState(
            MathUtil.clamp(pivotAngleRads,       PivotConstants.minAngle.in(Radians),            PivotConstants.maxAngle.in(Radians)),
            MathUtil.clamp(elevatorLengthMeters, ElevatorConstants.minLengthPhysical.in(Meters), ElevatorConstants.maxLengthPhysical.in(Meters)),
            MathUtil.clamp(wristAngleRads,       WristConstants.minAngle.in(Radians),            WristConstants.maxAngle.in(Radians))
        );
    }
    public static SuperstructureState newConstrained(Measure<AngleUnit> pivotAngle, Measure<DistanceUnit> elevatorLength, Measure<AngleUnit> wristAngle) {
        return SuperstructureState.newConstrained(pivotAngle.in(Radians), elevatorLength.in(Meters), wristAngle.in(Radians));
    }

    public static SuperstructureState fromParts(double pivotAngleRads, double elevatorLengthMeters, double wristAngleRads) {
        return SuperstructureState.newConstrained(pivotAngleRads, elevatorLengthMeters, wristAngleRads - pivotAngleRads);
    }
    public static SuperstructureState fromParts(Measure<AngleUnit> pivotAngle, Measure<DistanceUnit> elevatorLength, Measure<AngleUnit> wristAngle) {
        return SuperstructureState.fromParts(pivotAngle.in(Radians), elevatorLength.in(Meters), wristAngle.in(Radians));
    }

    public static SuperstructureState fromWristAxisPivotSpace(Transform2d pivotSpacePose) {
        var pivotToTargetMeters = pivotSpacePose.getTranslation().getNorm();
        var elevatorPivotOffsetMeters = ElevatorConstants.pivotOffset.in(Meters);

        var pivotAngleOffsetRads = Math.asin(elevatorPivotOffsetMeters / pivotToTargetMeters);

        var pivotAngleRads = Math.atan2(pivotSpacePose.getTranslation().getY(), pivotSpacePose.getTranslation().getX()) - pivotAngleOffsetRads;
        var wristAngleRads = MathUtil.angleModulus(pivotSpacePose.getRotation().getRadians() - pivotAngleRads);
        var elevatorHeightMeters = Math.sqrt((pivotToTargetMeters * pivotToTargetMeters) - (elevatorPivotOffsetMeters * elevatorPivotOffsetMeters)) - ElevatorConstants.stage2Base.getTranslation().getX();
        var elevatorLengthMeters = elevatorHeightMeters - ElevatorConstants.minHeightPhysical.in(Meters);

        return SuperstructureState.newConstrained(pivotAngleRads, elevatorLengthMeters, wristAngleRads);
    }
    public static SuperstructureState fromCoralTipPivotSpace(Transform2d coralTipPivotSpace) {
        return SuperstructureState.fromWristAxisPivotSpace(coralTipPivotSpace.plus(SuperstructureConstants.coralTipToWristAxis));
    }
    public static SuperstructureState fromAlgaeCenterPivotSpace(Transform2d algaeCenterPivotSpace) {
        return SuperstructureState.fromWristAxisPivotSpace(algaeCenterPivotSpace.plus(SuperstructureConstants.algaeCenterToWristAxis));
    }

    public static SuperstructureState fromWristAxisRobotSpace(Pose2d robotSpacePose) {
        return SuperstructureState.fromWristAxisPivotSpace(robotSpacePose.minus(PivotConstants.pivotRobotSpace));
    }
    public static SuperstructureState fromCoralTipRobotSpace(Pose2d coralTipRobotSpace) {
        return SuperstructureState.fromCoralTipPivotSpace(coralTipRobotSpace.minus(PivotConstants.pivotRobotSpace));
    }
    public static SuperstructureState fromAlgaeCenterRobotSpace(Pose2d algaeCenterRobotSpace) {
        return SuperstructureState.fromAlgaeCenterPivotSpace(algaeCenterRobotSpace.minus(PivotConstants.pivotRobotSpace));
    }

    public double getPivotAngleRads() {
        return this.pivotAngleRads;
    }
    public void setPivotAngleRads(double pivotAngleRads) {
        this.pivotAngleRads = pivotAngleRads;
    }

    public double getElevatorLengthMeters() {
        return this.elevatorLengthMeters;
    }
    public void setElevatorLengthMeters(double elevatorLengthMeters) {
        this.elevatorLengthMeters = elevatorLengthMeters;
    }

    public double getWristAngleRads() {
        return this.wristAngleRads;
    }
    public void setWristAngleRads(double wristAngleRads) {
        this.wristAngleRads = wristAngleRads;
    }

    public boolean isNear(SuperstructureState other, double pivotToleranceRads, double elevatorToleranceMeters, double wristToleranceRads) {
        return
            MathUtil.isNear(other.getPivotAngleRads(),       this.getPivotAngleRads(),       pivotToleranceRads) &&
            MathUtil.isNear(other.getElevatorLengthMeters(), this.getElevatorLengthMeters(), elevatorToleranceMeters) &&
            MathUtil.isNear(other.getWristAngleRads(),       this.getWristAngleRads(),       wristToleranceRads)
        ;
    }
    public boolean isNear(SuperstructureState other, Measure<AngleUnit> pivotTolerance, Measure<DistanceUnit> elevatorTolerance, Measure<AngleUnit> wristTolerance) {
        return this.isNear(other, pivotTolerance.in(Radians), elevatorTolerance.in(Meters), wristTolerance.in(Radians));
    }

    public Transform3d[] getMechTransforms() {
        var pivotMechTransform = new Transform3d(Translation3d.kZero, new Rotation3d(0.0, -this.getPivotAngleRads(), 0.0));
        var elevatorMechTransform = new Transform3d(new Translation3d(this.getElevatorLengthMeters() / ElevatorConstants.movingStageCount, 0.0, 0.0), Rotation3d.kZero);
        var wristMechTransform = new Transform3d(Translation3d.kZero, new Rotation3d(0.0, -this.getWristAngleRads(), 0.0));
        var pivotTransform = PivotConstants.pivotBase.plus(pivotMechTransform);
        var stage2Transform = pivotTransform.plus(ElevatorConstants.stage2Base).plus(elevatorMechTransform);
        var stage3Transform = stage2Transform.plus(ElevatorConstants.stage3Base).plus(elevatorMechTransform);
        var stage4Transform = stage3Transform.plus(ElevatorConstants.stage4Base).plus(elevatorMechTransform);
        var wristTransform = stage4Transform.plus(WristConstants.wristBase).plus(wristMechTransform);
        return new Transform3d[] {
            pivotTransform,
            stage2Transform,
            stage3Transform,
            stage4Transform,
            wristTransform,
        };
    }
}
