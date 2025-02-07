package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import frc.util.robotStructure.PointOfMass;

public class Superstructure extends SubsystemBase {
    public final Pivot pivot;
    public final Elevator elevator;
    public final Wrist wrist;
    public Translation2d centerOfMass = Translation2d.kZero;

    public Superstructure(Pivot pivot, Elevator elevator, Wrist wrist) {
        System.out.println("[Init Superstructure] Instantiating Superstructure");
        this.pivot = pivot;
        this.elevator = elevator;
        this.wrist = wrist;
    }

    @Override
    public void periodic() {
        pivot.periodic();
        elevator.periodic();
        wrist.periodic();

        var centerOfMass3d = PointOfMass.getCenterOfMass(
            pivot.stage1Mass,
            elevator.stage2Mass,
            elevator.stage3Mass,
            elevator.stage4Mass,
            wrist.wristMass
        );

        centerOfMass = new Translation2d(
            centerOfMass3d.getX(),
            centerOfMass3d.getZ()
        );

        Logger.recordOutput("Center of Mass", new Pose3d(RobotState.getInstance().getPose()).transformBy(new Transform3d(centerOfMass3d, Rotation3d.kZero)));
    }

    public SuperstructureState getCurrentState() {
        return new SuperstructureState(
            pivot.getAngle(),
            elevator.getLength(),
            wrist.getAngle()
        );
    }

    public Command idle() {
        return goToSetpointSequenced(SuperstructureState.idle);
    }

    public Command defense() {
        return goToSetpointSequenced(SuperstructureState.defense);
    }

    public Command pivotVoltage(DoubleSupplier voltage) {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
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
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
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

    public static final Transform2d forwardCoralTransform = new Transform2d(
        new Translation2d(
            Coral.length.plus(Inches.of(1)),
            Inches.zero()
        ),
        new Rotation2d(Degrees.of(35).unaryMinus())
    ).inverse();
    public static final Transform2d backwardCoralTransform = new Transform2d(
        new Translation2d(
            Coral.length.plus(Inches.of(1)),
            Inches.zero()
        ),
        new Rotation2d(Degrees.of(35))
    ).inverse();
    public static final Transform2d algaeTransform = new Transform2d(
        new Translation2d(
            Algae.radius.plus(Inches.of(1)),
            Inches.zero()
        ),
        Rotation2d.kZero
    ).inverse();

    // public Command goToSetpoint(SuperstructureState setpoint) {
    //     return Commands.parallel(
    //         pivot.pivotTo(setpoint.pivotAngle),
    //         elevator.elevateTo(setpoint.elevatorLength),
    //         wrist.pivotTo(setpoint.wristAngle)
    //     );
    // }

    public Command goToSetpointSequenced(SuperstructureState setpoint) {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("Superstructure Setpoint Sequenced");
            }
            private SuperstructureState initialState;
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
                initialState = getCurrentState();
                var elevatorFirst = setpoint.elevatorLength.lt(initialState.elevatorLength);
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
                    pivot.setPivot(initialState.pivotAngle);
                }
                if (elevatorMoving) {
                    elevator.setLength(setpoint.elevatorLength);
                } else {
                    elevator.setLength(initialState.elevatorLength);
                }
                if (wristMoving) {
                    wrist.setAngle(setpoint.wristAngle);
                } else {
                    wrist.setAngle(initialState.wristAngle);
                }
                Logger.recordOutput("Superstructure/Setpoint/Pivot Moving", pivotMoving);
                Logger.recordOutput("Superstructure/Setpoint/Elevator Moving", elevatorMoving);
                Logger.recordOutput("Superstructure/Setpoint/Wrist Moving", wristMoving);
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

    public static class SuperstructureState {
        public final Angle pivotAngle;
        public final Distance elevatorLength;
        public final Angle wristAngle;

        public static final SuperstructureState idle = new SuperstructureState(
            Degrees.of(70),
            Meters.zero(),
            Degrees.of(90).minus(Degrees.of(70))
        );
        public static final SuperstructureState defense = new SuperstructureState(
            PivotConstants.minAngle,
            Meters.zero(),
            Degrees.of(110).minus(PivotConstants.minAngle)
        );

        public SuperstructureState(Angle pivotAngle, Distance elevatorLength, Angle wristAngle) {
            this.pivotAngle = pivotAngle;
            this.elevatorLength = elevatorLength;
            this.wristAngle = wristAngle;
        }

        public static SuperstructureState fromPivotSpace(Pose2d pivotSpacePose) {
            var pivotToTargetDist = Meters.of(pivotSpacePose.getTranslation().getNorm());

            var pivotAngleOffset = Radians.of(Math.asin(ElevatorConstants.pivotOffset.div(pivotToTargetDist).baseUnitMagnitude()));

            var pivotAngle = pivotSpacePose.getTranslation().getAngle().getMeasure().minus(pivotAngleOffset);
            var wristAngle = pivotSpacePose.getRotation().minus(new Rotation2d(pivotAngle)).getMeasure();
            var pivotToTargetMeters = pivotToTargetDist.in(Meters);
            var elevatorPivotOffsetMeters = ElevatorConstants.pivotOffset.in(Meters);
            var elevatorHeight = Meters.of(Math.sqrt((pivotToTargetMeters * pivotToTargetMeters) - (elevatorPivotOffsetMeters * elevatorPivotOffsetMeters))).plus(ElevatorConstants.stage2Base.getMeasureX().unaryMinus());
            var elevatorLength = elevatorHeight.minus(ElevatorConstants.minimumHeight);

            return new SuperstructureState(pivotAngle, elevatorLength, wristAngle);
        }

        public static SuperstructureState fromRobotSpace(Pose2d robotSpacePose) {
            return fromPivotSpace(robotSpacePose.relativeTo(PivotConstants.pivotRobotSpace));
        }
        public static SuperstructureState fromLevelForward(Level level) {
            return fromRobotSpace(level.forwardBranchRobotSpace.transformBy(forwardCoralTransform));
        }
        public static SuperstructureState fromLevelBackward(Level level) {
            return fromRobotSpace(level.backwardBranchRobotSpace.transformBy(backwardCoralTransform));
        }
        public static SuperstructureState fromAlgaeForward(AlgaeLevel algaeLevel) {
            return fromRobotSpace(algaeLevel.forwardRobotSpace.transformBy(algaeTransform));
        }
        public static SuperstructureState fromAlgaeBackward(AlgaeLevel algaeLevel) {
            return fromRobotSpace(algaeLevel.backwardRobotSpace.transformBy(algaeTransform));
        }
    }
}
