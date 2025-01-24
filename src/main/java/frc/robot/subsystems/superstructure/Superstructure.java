package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants.Algae;
import frc.robot.constants.FieldConstants.Coral;
import frc.robot.constants.FieldConstants.Reef.AlgaeLevel;
import frc.robot.constants.FieldConstants.Reef.Level;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.PivotConstants;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.util.misc.MeasureUtil;

public class Superstructure {
    public final Pivot pivot;
    public final Elevator elevator;
    public final Wrist wrist;
    public final Set<Subsystem> subsystems;

    public Superstructure(Pivot pivot, Elevator elevator, Wrist wrist) {
        this.pivot = pivot;
        this.elevator = elevator;
        this.wrist = wrist;
        this.subsystems = Set.of(pivot, elevator, wrist);
    }

    public Command pivotVoltage(DoubleSupplier voltage) {
        return new Command() {
            {
                addRequirements(pivot, elevator);
                setName("Pivot Voltage");
            }
            @Override
            public void initialize() {

            }
            @Override
            public void execute() {
                pivot.setVoltage(Volts.of(voltage.getAsDouble()));
                elevator.setVoltage(Volts.zero());
            }
        };
    }
    public Command elevatorVoltage(DoubleSupplier voltage) {
        return new Command() {
            {
                addRequirements(pivot, elevator);
                setName("Elevator Voltage");
            }
            @Override
            public void initialize() {

            }
            @Override
            public void execute() {
                pivot.setVoltage(Volts.zero());
                elevator.setVoltage(Volts.of(voltage.getAsDouble()));
            }
        };
    }

    private static final Transform2d forwardCoralTransform = new Transform2d(
        new Translation2d(
            Coral.length.plus(Inches.of(1)),
            Inches.zero()
        ),
        new Rotation2d(Degrees.of(35).unaryMinus())
    ).inverse();
    private static final Transform2d backwardCoralTransform = new Transform2d(
        new Translation2d(
            Coral.length.plus(Inches.of(1)),
            Inches.zero()
        ),
        new Rotation2d(Degrees.of(35))
    ).inverse();
    private static final Transform2d algaeTransform = new Transform2d(
        new Translation2d(
            Algae.radius.plus(Inches.of(1)),
            Inches.zero()
        ),
        Rotation2d.kZero
    ).inverse();

    public Command inverseKinematics(Pose2d target) {
        var pivotToTargetDist = Meters.of(target.getTranslation().getNorm());

        var pivotAngleOffset = Radians.of(Math.asin(ElevatorConstants.pivotOffset.div(pivotToTargetDist).baseUnitMagnitude()));

        var pivotAngle = target.getTranslation().getAngle().getMeasure().minus(pivotAngleOffset);
        var wristAngle = target.getRotation().minus(new Rotation2d(pivotAngle)).getMeasure();
        var pivotToTargetMeters = pivotToTargetDist.in(Meters);
        var elevatorPivotOffsetMeters = ElevatorConstants.pivotOffset.in(Meters);
        var elevatorHeight = Meters.of(Math.sqrt((pivotToTargetMeters * pivotToTargetMeters) - (elevatorPivotOffsetMeters * elevatorPivotOffsetMeters))).plus(ElevatorConstants.elevatorBase.getMeasureX().unaryMinus());
        var elevatorLength = elevatorHeight.minus(ElevatorConstants.minimumHeight);

        return Commands.parallel(
            Commands.run(() -> {
                Logger.recordOutput("Superstructure/Inverse Kinematics/Target", target);
                Logger.recordOutput("Superstructure/Inverse Kinematics/Pivot Angle", pivotAngle);
                Logger.recordOutput("Superstructure/Inverse Kinematics/Elevator Length", elevatorLength);
                Logger.recordOutput("Superstructure/Inverse Kinematics/Wrist Angle", wristAngle);
                var transform = new Transform3d(
                    new Translation3d(
                        target.getX(),
                        0,
                        target.getY()
                    ),
                    new Rotation3d(
                        0,
                        -target.getRotation().getRadians(),
                        0
                    )
                );
                Logger.recordOutput("Superstructure/Inverse Kinematics/3d Target", new Pose3d(RobotState.getInstance().getPose()).transformBy(PivotConstants.pivotBase).transformBy(transform));
            }),
            pivot.pivotTo(pivotAngle),
            elevator.elevateTo(elevatorLength),
            wrist.pivotTo(wristAngle)
        );
    }

    public Command goToSetpoint(SuperstructureSetpoint setpoint) {
        return Commands.parallel(
            pivot.pivotTo(setpoint.pivotAngle),
            elevator.elevateTo(setpoint.elevatorLength),
            wrist.pivotTo(setpoint.wristAngle)
        );
    }

    public Command goToSetpointSequenced(SuperstructureSetpoint setpoint) {
        return new Command() {
            {
                addRequirements(subsystems);
                setName("Superstructure Setpoint With Safety");
            }
            private Angle initialPivotAngle;
            private Distance initialElevatorLength;
            private Angle initialWristAngle;
            private BooleanSupplier pivotMoveCondition;
            private BooleanSupplier elevatorMoveCondition;
            private BooleanSupplier wristMoveCondition;
            private boolean pivotMoving;
            private boolean elevatorMoving;
            private boolean wristMoving;
            @Override
            public void initialize() {
                pivotMoving = false;
                elevatorMoving = false;
                wristMoving = false;
                initialPivotAngle = pivot.getAngle();
                initialElevatorLength = elevator.getLength();
                initialWristAngle = wrist.getAngle();
                var elevatorFirst = setpoint.elevatorLength.lt(initialElevatorLength);
                pivotMoveCondition = () -> !elevatorFirst || MeasureUtil.isNear(setpoint.elevatorLength, elevator.getLength(), Inches.of(5));
                elevatorMoveCondition = () -> elevatorFirst || MeasureUtil.isNear(setpoint.pivotAngle, pivot.getAngle(), Degrees.of(5));
                wristMoveCondition = () -> true;
            }
            @Override
            public void execute() {
                if (!pivotMoving) {
                    pivotMoving = pivotMoveCondition.getAsBoolean();
                }
                if (!elevatorMoving) {
                    elevatorMoving = elevatorMoveCondition.getAsBoolean();
                }
                if (!wristMoving) {
                    wristMoving = wristMoveCondition.getAsBoolean();
                }
                if (pivotMoving) {
                    pivot.setPivot(setpoint.pivotAngle);
                } else {
                    pivot.setPivot(initialPivotAngle);
                }
                if (elevatorMoving) {
                    elevator.setLength(setpoint.elevatorLength);
                } else {
                    elevator.setLength(initialElevatorLength);
                }
                if (wristMoving) {
                    wrist.setAngle(setpoint.wristAngle);
                } else {
                    wrist.setAngle(initialWristAngle);
                }
                Logger.recordOutput("Superstructure/Pivot Moving", pivotMoving);
                Logger.recordOutput("Superstructure/Elevator Moving", elevatorMoving);
                Logger.recordOutput("Superstructure/Wrist Moving", wristMoving);
            }
            @Override
            public void end(boolean interrupted) {
                
            }
            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    public static class SuperstructureSetpoint {
        public final Angle pivotAngle;
        public final Distance elevatorLength;
        public final Angle wristAngle;

        public SuperstructureSetpoint(Angle pivotAngle, Distance elevatorLength, Angle wristAngle) {
            this.pivotAngle = pivotAngle;
            this.elevatorLength = elevatorLength;
            this.wristAngle = wristAngle;
        }

        public static SuperstructureSetpoint fromPivotSpace(Pose2d pivotSpacePose) {
            var pivotToTargetDist = Meters.of(pivotSpacePose.getTranslation().getNorm());

            var pivotAngleOffset = Radians.of(Math.asin(ElevatorConstants.pivotOffset.div(pivotToTargetDist).baseUnitMagnitude()));

            var pivotAngle = pivotSpacePose.getTranslation().getAngle().getMeasure().minus(pivotAngleOffset);
            var wristAngle = pivotSpacePose.getRotation().minus(new Rotation2d(pivotAngle)).getMeasure();
            var pivotToTargetMeters = pivotToTargetDist.in(Meters);
            var elevatorPivotOffsetMeters = ElevatorConstants.pivotOffset.in(Meters);
            var elevatorHeight = Meters.of(Math.sqrt((pivotToTargetMeters * pivotToTargetMeters) - (elevatorPivotOffsetMeters * elevatorPivotOffsetMeters))).plus(ElevatorConstants.elevatorBase.getMeasureX().unaryMinus());
            var elevatorLength = elevatorHeight.minus(ElevatorConstants.minimumHeight);

            return new SuperstructureSetpoint(pivotAngle, elevatorLength, wristAngle);
        }

        public static final Pose2d pivotRobotSpace = new Pose2d(
            new Translation2d(
                PivotConstants.pivotBase.getMeasureX(),
                PivotConstants.pivotBase.getMeasureZ()
            ),
            Rotation2d.kZero
        );

        public static SuperstructureSetpoint fromRobotSpace(Pose2d robotSpacePose) {
            return fromPivotSpace(robotSpacePose.relativeTo(pivotRobotSpace));
        }
        public static SuperstructureSetpoint fromLevelForward(Level level) {
            return fromRobotSpace(level.forwardBranchRobotSpace.transformBy(forwardCoralTransform));
        }
        public static SuperstructureSetpoint fromLevelBackward(Level level) {
            return fromRobotSpace(level.backwardBranchRobotSpace.transformBy(backwardCoralTransform));
        }
        public static SuperstructureSetpoint fromAlgaeForward(AlgaeLevel algaeLevel) {
            return fromRobotSpace(algaeLevel.forwardRobotSpace.transformBy(algaeTransform));
        }
        public static SuperstructureSetpoint fromAlgaeBackward(AlgaeLevel algaeLevel) {
            return fromRobotSpace(algaeLevel.backwardRobotSpace.transformBy(algaeTransform));
        }
    }
}
