package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants.Coral;
import frc.robot.constants.FieldConstants.Reef.Level;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.PivotConstants;

public class Superstructure {
    public final Pivot pivot;
    public final Elevator elevator;

    public Superstructure(Pivot pivot, Elevator elevator) {
        this.pivot = pivot;
        this.elevator = elevator;
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

    private static final Transform2d forwardScoringTransform = new Transform2d(
        new Translation2d(
            Coral.length.plus(Inches.of(1)),
            Inches.zero()
        ),
        new Rotation2d(Degrees.of(35).unaryMinus())
    ).inverse();
    private static final Transform2d backwardScoringTransform = new Transform2d(
        new Translation2d(
            Coral.length.plus(Inches.of(1)),
            Inches.zero()
        ),
        new Rotation2d(Degrees.of(35))
    ).inverse();

    public Command goToLevelForward(Level level) {
        return inverseKinematics(level.forwardBranch.transformBy(forwardScoringTransform));
    }
    public Command goToLevelBackward(Level level) {
        return inverseKinematics(level.backwardBranch.transformBy(backwardScoringTransform));
    }

    public Command inverseKinematics(Pose2d target) {
        var pivotToTargetDist = Meters.of(target.getTranslation().getNorm());

        var pivotAngleOffset = Radians.of(Math.asin(ElevatorConstants.pivotOffset.div(pivotToTargetDist).baseUnitMagnitude()));

        var pivotAngle = target.getTranslation().getAngle().getMeasure().minus(pivotAngleOffset);
        var wristAngle = target.getRotation().getMeasure().minus(pivotAngle);
        var pivotToTargetMeters = pivotToTargetDist.in(Meters);
        var elevatorPivotOffsetMeters = ElevatorConstants.pivotOffset.in(Meters);
        var elevatorHeight = Meters.of(Math.sqrt((pivotToTargetMeters * pivotToTargetMeters) - (elevatorPivotOffsetMeters * elevatorPivotOffsetMeters)));
        var elevatorLength = elevatorHeight.minus(ElevatorConstants.minimumHeight);

        // wrist.pivotTo(wristAngle);
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
            elevator.elevateTo(elevatorLength)
        );
    }
}
