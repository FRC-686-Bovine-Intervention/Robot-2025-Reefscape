package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.PivotConstants;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.flipping.AllianceFlipUtil;
import frc.util.flipping.AllianceFlipUtil.FieldFlipType;
import frc.util.flipping.AllianceFlippable;
import frc.util.geometry.GeomUtil;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;

public class Superstructure extends SubsystemBase {
    public final Pivot pivot;
    public final Elevator elevator;
    public final Wrist wrist;

    private final SuperstructureState measuredState = SuperstructureState.newUnconstrained(0.0, 0.0, 0.0);

    public Superstructure(Pivot pivot, Elevator elevator, Wrist wrist) {
        System.out.println("[Init Superstructure] Instantiating Superstructure");
        this.pivot = pivot;
        this.elevator = elevator;
        this.wrist = wrist;

        var pivotRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).div(Seconds.of(2)),
                Volts.of(5),
                Seconds.of(15),
                (state) -> {
                    Logger.recordOutput("Superstructure/Pivot/SysID/State", state.toString());
                }
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                    this.pivot.setVolts(voltage.in(Volts));
                    this.elevator.setLengthGoalMeters(ElevatorConstants.minLengthPhysical.in(Meters));
                    this.wrist.setAngleGoalRads(0);
                },
                (log) -> {
                    Logger.recordOutput("Superstructure/Pivot/SysID/Voltage", this.pivot.getAppliedVolts());
                    Logger.recordOutput("Superstructure/Pivot/SysID/Position", this.pivot.getAngleRads());
                    Logger.recordOutput("Superstructure/Pivot/SysID/Velocity", this.pivot.getVelocityRadsPerSec());
                },
                this,
                "Pivot"
            )
        );
        SmartDashboard.putData("SysID/Superstructure/Pivot/Quasi Forward", pivotRoutine.quasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("SysID/Superstructure/Pivot/Quasi Reverse", pivotRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("SysID/Superstructure/Pivot/Dynamic Forward", pivotRoutine.dynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("SysID/Superstructure/Pivot/Dynamic Reverse", pivotRoutine.dynamic(SysIdRoutine.Direction.kReverse));
        var elevatorRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).div(Seconds.of(2)),
                Volts.of(5),
                Seconds.of(15),
                (state) -> {
                    Logger.recordOutput("Superstructure/Elevator/SysID/State", state.toString());
                }
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                    this.pivot.setAngleGoalRads(90);
                    this.elevator.setVolts(voltage.in(Volts));
                    this.wrist.setAngleGoalRads(0);
                },
                (log) -> {
                    Logger.recordOutput("Superstructure/Elevator/SysID/Voltage", this.elevator.getAppliedVolts());
                    Logger.recordOutput("Superstructure/Elevator/SysID/Position", this.elevator.getLengthMeters());
                    Logger.recordOutput("Superstructure/Elevator/SysID/Velocity", this.elevator.getVelocityMetersPerSec());
                },
                this,
                "Elevator"
            )
        );
        SmartDashboard.putData("SysID/Superstructure/Elevator/Quasi Forward", elevatorRoutine.quasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("SysID/Superstructure/Elevator/Quasi Reverse", elevatorRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("SysID/Superstructure/Elevator/Dynamic Forward", elevatorRoutine.dynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("SysID/Superstructure/Elevator/Dynamic Reverse", elevatorRoutine.dynamic(SysIdRoutine.Direction.kReverse));
        var wristRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).div(Seconds.of(2)),
                Volts.of(3),
                Seconds.of(15),
                (state) -> {
                    Logger.recordOutput("Superstructure/Wrist/SysID/State", state.toString());
                }
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                    this.pivot.setAngleGoalRads(90);
                    this.elevator.setLengthGoalMeters(ElevatorConstants.minLengthPhysical.in(Meters));
                    this.wrist.setVolts(voltage.in(Volts));
                },
                (log) -> {
                    Logger.recordOutput("Superstructure/Wrist/SysID/Voltage", this.wrist.getAppliedVolts());
                    Logger.recordOutput("Superstructure/Wrist/SysID/Position", this.wrist.getAngleRads());
                    Logger.recordOutput("Superstructure/Wrist/SysID/Velocity", this.wrist.getVelocityRadsPerSec());
                },
                this,
                "Wrist"
            )
        );
        SmartDashboard.putData("SysID/Superstructure/Wrist/Quasi Forward", wristRoutine.quasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("SysID/Superstructure/Wrist/Quasi Reverse", wristRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("SysID/Superstructure/Wrist/Dynamic Forward", wristRoutine.dynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("SysID/Superstructure/Wrist/Dynamic Reverse", wristRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    @Override
    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Superstructure/Before");
        this.pivot.periodic();
        this.elevator.periodic();
        this.wrist.periodic();
        this.measuredState.setPivotAngleRads(this.pivot.getAngleRads());
        this.measuredState.setElevatorLengthMeters(this.elevator.getLengthMeters());
        this.measuredState.setWristAngleRads(this.wrist.getAngleRads());
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Superstructure");
    }

    public SuperstructureState getCurrentMeasuredState() {
        return this.measuredState;
    }

    public Command throttle(DoubleSupplier pivotThrottle, DoubleSupplier elevatorThrottle, DoubleSupplier wristThrottle) {
        final var superstructure = this;
        return new Command() {
            private static final LoggedTunableMeasure<VoltageUnit> pivotVoltage = new LoggedTunableMeasure<>("Superstructure/Pivot Voltage", Volts.of(2));
            private static final LoggedTunableMeasure<VoltageUnit> elevatorVoltage = new LoggedTunableMeasure<>("Superstructure/Elevator Voltage", Volts.of(2));
            private static final LoggedTunableMeasure<VoltageUnit> wristVoltage = new LoggedTunableMeasure<>("Superstructure/Wrist Voltage", Volts.of(2));
            {
                this.addRequirements(superstructure);
                this.setName("Throttle");
            }

            @Override
            public void initialize() {
                
            }
            @Override
            public void execute() {
                superstructure.pivot.setVolts(pivotVoltage.get().in(Volts) * pivotThrottle.getAsDouble());
                superstructure.elevator.setVolts(elevatorVoltage.get().in(Volts) * elevatorThrottle.getAsDouble());
                superstructure.wrist.setVolts(wristVoltage.get().in(Volts) * wristThrottle.getAsDouble());
            }
            @Override
            public void end(boolean interrupted) {
                superstructure.pivot.stop(NeutralMode.DEFAULT);
                superstructure.elevator.stop(NeutralMode.DEFAULT);
                superstructure.wrist.stop(NeutralMode.DEFAULT);
            }
        };
    }

    public Command coast() {
        final var superstructure = this;
        return new Command() {
            {
                this.addRequirements(superstructure);
                this.setName("Coast");
            }
            @Override
            public void initialize() {
                superstructure.pivot.stop(NeutralMode.COAST);
                superstructure.elevator.stop(NeutralMode.COAST);
                superstructure.wrist.stop(NeutralMode.COAST);
            }
            @Override
            public void end(boolean interrupted) {
                superstructure.pivot.stop(NeutralMode.DEFAULT);
                superstructure.elevator.stop(NeutralMode.DEFAULT);
                superstructure.wrist.stop(NeutralMode.DEFAULT);
            }
            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        };
    }

    public Command directToSetpoint(SuperstructureState setpoint) {
        final var superstructure = this;
        return new Command() {
            {
                this.addRequirements(superstructure);
                this.setName("Direct to Setpoint");
            }
            @Override
            public void execute() {
                superstructure.pivot.setAngleGoalRads(setpoint.getPivotAngleRads());
                superstructure.elevator.setLengthGoalMeters(setpoint.getElevatorLengthMeters());
                superstructure.wrist.setAngleGoalRads(setpoint.getWristAngleRads());
            }
            @Override
            public void end(boolean interrupted) {
                superstructure.pivot.stop(NeutralMode.DEFAULT);
                superstructure.elevator.stop(NeutralMode.DEFAULT);
                superstructure.wrist.stop(NeutralMode.DEFAULT);
            }
        };
    }

    public Command goToSetpointSequenced(SuperstructureState setpointState) {
        final var superstructure = this;
        return new Command() {
            {
                this.addRequirements(superstructure);
                this.setName("Superstructure Setpoint Sequenced");
            }
            private final ArrayList<SuperstructureStep> steps = new ArrayList<>(4);
            private int currentStepIndex = 0;
            @Override
            public void initialize() {
                this.currentStepIndex = 0;
                this.steps.clear();
                final var initialState = superstructure.getCurrentMeasuredState();

                final var initialVeryLow =   initialState.getElevatorLengthMeters() < +Units.inchesToMeters(13);
                final var initialLow =       initialState.getElevatorLengthMeters() < +Units.inchesToMeters(25);
                final var initialHigh =      initialState.getElevatorLengthMeters() > +Units.inchesToMeters(45);
                final var initialWristUp =   initialState.getWristAngleRads()       > +Units.degreesToRadians(45);
                final var initialWristDown = initialState.getWristAngleRads()       < -Units.degreesToRadians(60);
                final var initialClimbing =  initialState.getWristAngleRads() > +Units.degreesToRadians(70) && initialState.getPivotAngleRads() > +Units.degreesToRadians(90);
                
                final var targetVeryLow =    setpointState.getElevatorLengthMeters() < +Units.inchesToMeters(13);
                final var targetLow =        setpointState.getElevatorLengthMeters() < +Units.inchesToMeters(25);
                final var targetHigh =       setpointState.getElevatorLengthMeters() > +Units.inchesToMeters(45);
                final var targetWristDown =  setpointState.getWristAngleRads()       < -Units.degreesToRadians(60);

                Logger.recordOutput("Superstructure/Sequencing/initialVeryLow", initialVeryLow);
                Logger.recordOutput("Superstructure/Sequencing/initialLow", initialLow);
                Logger.recordOutput("Superstructure/Sequencing/initialHigh", initialHigh);
                Logger.recordOutput("Superstructure/Sequencing/initialWristUp", initialWristUp);
                Logger.recordOutput("Superstructure/Sequencing/initialWristDown", initialWristDown);
                Logger.recordOutput("Superstructure/Sequencing/initialClimbing", initialClimbing);
                Logger.recordOutput("Superstructure/Sequencing/targetVeryLow", targetVeryLow);
                Logger.recordOutput("Superstructure/Sequencing/targetLow", targetLow);
                Logger.recordOutput("Superstructure/Sequencing/targetHigh", targetHigh);
                Logger.recordOutput("Superstructure/Sequencing/targetWristDown", targetWristDown);

                if (initialClimbing) {

                } else if (initialLow && initialWristUp) { // Remove Coral from station
                    this.steps.add(
                        new SuperstructureStep(
                            SuperstructureState.newConstrained(
                                Units.degreesToRadians(60),
                                ElevatorConstants.minLengthPhysical.in(Meters),
                                initialState.getWristAngleRads()
                            ),
                            Units.degreesToRadians(7.5),
                            Units.inchesToMeters(5),
                            Units.degreesToRadians(15)
                        )
                    );
                }

                if (initialLow && targetHigh) { // Extend Safely
                    this.steps.add(
                        new SuperstructureStep(
                            SuperstructureState.fromParts(
                                MathUtil.clamp(setpointState.getPivotAngleRads(), Units.degreesToRadians(30), Units.degreesToRadians(110)),
                                initialState.getElevatorLengthMeters(),
                                MathUtil.clamp(setpointState.getWristAngleRads(), Units.degreesToRadians(80), Units.degreesToRadians(90))
                            ),
                            Units.degreesToRadians(10),
                            Units.inchesToMeters(10),
                            Units.degreesToRadians(5)
                        )
                    );
                    this.steps.add(
                        new SuperstructureStep(
                            SuperstructureState.fromParts(
                                MathUtil.clamp(setpointState.getPivotAngleRads(), Units.degreesToRadians(30), Units.degreesToRadians(110)),
                                setpointState.getElevatorLengthMeters(),
                                MathUtil.clamp(setpointState.getWristAngleRads(), Units.degreesToRadians(80), Units.degreesToRadians(90))
                            ),
                            Units.degreesToRadians(10),
                            Units.inchesToMeters(5),
                            Units.degreesToRadians(10)
                        )
                    );
                }
                if (targetVeryLow && initialVeryLow && !initialWristDown && targetWristDown) {
                    this.steps.add(
                        new SuperstructureStep(
                            SuperstructureState.newConstrained(
                                setpointState.getPivotAngleRads(),
                                initialState.getElevatorLengthMeters(),
                                setpointState.getWristAngleRads()
                            ),
                            Units.degreesToRadians(10),
                            Units.inchesToMeters(5),
                            Units.degreesToRadians(10)
                        )
                    );
                }

                if (initialHigh && targetLow) { // Retract Safely
                    this.steps.add(
                        new SuperstructureStep(
                            SuperstructureState.fromParts(
                                MathUtil.clamp(initialState.getPivotAngleRads(), Units.degreesToRadians(75), Units.degreesToRadians(90)),
                                initialState.getElevatorLengthMeters(),
                                Units.degreesToRadians(90)
                            ),
                            Units.degreesToRadians(10),
                            Units.inchesToMeters(10),
                            Units.degreesToRadians(5)
                        )
                    );
                    this.steps.add(
                        new SuperstructureStep(
                            SuperstructureState.fromParts(
                                MathUtil.clamp(initialState.getPivotAngleRads(), Units.degreesToRadians(75), Units.degreesToRadians(90)),
                                setpointState.getElevatorLengthMeters(),
                                Units.degreesToRadians(90)
                            ),
                            Units.degreesToRadians(10),
                            Units.inchesToMeters(10),
                            Units.degreesToRadians(10)
                        )
                    );
                }
                
                this.steps.add(new SuperstructureStep(setpointState, 0.0, 0.0, 0.0));
            }
            @Override
            public void execute() {
                var currentStep = this.steps.get(this.currentStepIndex);
                if (this.currentStepIndex < this.steps.size() - 1 && currentStep.closeToTarget(superstructure.getCurrentMeasuredState())) {
                    this.currentStepIndex += 1;
                    currentStep = this.steps.get(this.currentStepIndex);
                }
                var pivotSetpoint = currentStep.targetState.getPivotAngleRads();
                var elevatorSetpoint = currentStep.targetState.getElevatorLengthMeters();
                var wristSetpoint = currentStep.targetState.getWristAngleRads();
                superstructure.pivot.setAngleGoalRads(pivotSetpoint);
                superstructure.elevator.setLengthGoalMeters(elevatorSetpoint);
                superstructure.wrist.setAngleGoalRads(wristSetpoint);
                Logger.recordOutput("Superstructure/Setpoint/Pivot Setpoint", pivotSetpoint);
                Logger.recordOutput("Superstructure/Setpoint/Elevator Setpoint", elevatorSetpoint);
                Logger.recordOutput("Superstructure/Setpoint/Wrist Setpoint", wristSetpoint);
                Logger.recordOutput("Superstructure/Setpoint/Mech Transforms", currentStep.targetState.getMechTransforms());
            }
            @Override
            public void end(boolean interrupted) {
                superstructure.pivot.stop(NeutralMode.DEFAULT);
                superstructure.elevator.stop(NeutralMode.DEFAULT);
                superstructure.wrist.stop(NeutralMode.DEFAULT);
            }
        };
    }

    private static class SuperstructureStep {
        public final SuperstructureState targetState;
        public final double pivotToleranceRads;
        public final double elevatorToleranceMeters;
        public final double wristToleranceRads;

        public SuperstructureStep(SuperstructureState targetState, double pivotToleranceRads, double elevatorToleranceMeters, double wristToleranceRads) {
            this.targetState = targetState;
            this.pivotToleranceRads = pivotToleranceRads;
            this.elevatorToleranceMeters = elevatorToleranceMeters;
            this.wristToleranceRads = wristToleranceRads;
        }

        public boolean closeToTarget(SuperstructureState currentState) {
            return currentState.isNear(this.targetState, this.pivotToleranceRads, this.elevatorToleranceMeters, this.wristToleranceRads);
        }
    }

    public static enum Direction {
        Forward(true),
        Backward(false),
        ;
        private final boolean forward;
        Direction(boolean forward) {
            this.forward = forward;
        }
        public static Direction getClosest(Rotation2d target, Rotation2d current) {
            if (target.minus(current).getCos() >= 0) {
                return Direction.Forward;
            } else {
                return Direction.Backward;
            }
        }
        public boolean isForward() {
            return this.forward;
        }
        public boolean isBackward() {
            return !this.forward;
        }
    }

    public static class RobotFlippedRobotPose implements AllianceFlippable<RobotFlippedRobotPose> {
        private final Pose2d forward;
        private final Pose2d backward;

        public RobotFlippedRobotPose(Pose2d forward, Pose2d backward) {
            this.forward = forward;
            this.backward = backward;
        }

        public static RobotFlippedRobotPose fromForwardRobotFlipped(Pose2d forward) {
            return new RobotFlippedRobotPose(
                forward,
                forward.transformBy(GeomUtil.rotate180Transform2d)
            );
        }
        public static RobotFlippedRobotPose fromBackwardRobotFlipped(Pose2d backward) {
            return new RobotFlippedRobotPose(
                backward.transformBy(GeomUtil.rotate180Transform2d),
                backward
            );
        }
        public static RobotFlippedRobotPose fromForwardPivotFlipped(Pose2d forward) {
            return new RobotFlippedRobotPose(
                forward,
                forward.transformBy(new Transform2d(
                    new Translation2d(
                        PivotConstants.pivotRobotSpace.getMeasureX().times(2),
                        Meters.zero()
                    ),
                    Rotation2d.k180deg
                ))
            );
        }
        public static RobotFlippedRobotPose fromBackwardPivotFlipped(Pose2d backward) {
            return new RobotFlippedRobotPose(
                backward.transformBy(new Transform2d(
                    new Translation2d(
                        PivotConstants.pivotRobotSpace.getMeasureX().times(2),
                        Meters.zero()
                    ),
                    Rotation2d.k180deg
                )),
                backward
            );
        }
        public static RobotFlippedRobotPose fromForwardOnly(Pose2d forward) {
            return new RobotFlippedRobotPose(forward, null);
        }
        public static RobotFlippedRobotPose fromBackwardOnly(Pose2d backward) {
            return new RobotFlippedRobotPose(null, backward);
        }

        public Pose2d getForward() {
            return forward;
        }
        public Pose2d getBackward() {
            return backward;
        }
        public Pose2d get(Direction direction) {
            switch (direction) {
                default:
                case Forward:   return getForward();
                case Backward:  return getBackward();
            }
        }

        public Direction getClosestDirection(Rotation2d current) {
            if (forward == null) {
                return Direction.Backward;
            } else if (backward == null) {
                return Direction.Forward;
            } else {
                return Direction.getClosest(forward.getRotation(), current);
            }
        }

        public Pose2d getClosest(Rotation2d current) {
            return get(getClosestDirection(current));
        }

        @Override
        public RobotFlippedRobotPose flip(FieldFlipType flipType) {
            return new RobotFlippedRobotPose(
                (this.forward == null) ? null : AllianceFlipUtil.flip(this.forward, flipType),
                (this.backward == null) ? null : AllianceFlipUtil.flip(this.backward, flipType)
            );
        }
    }
}
