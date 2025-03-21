package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstants;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.PivotConstants;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.subsystems.superstructure.wrist.WristConstants;
import frc.util.flipping.AllianceFlipUtil;
import frc.util.flipping.AllianceFlipUtil.FieldFlipType;
import frc.util.geometry.GeomUtil;
import frc.util.flipping.AllianceFlippable;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;
import frc.util.misc.MeasureUtil;

public class Superstructure extends SubsystemBase {
    public final Pivot pivot;
    public final Elevator elevator;
    public final Wrist wrist;

    // public final LoggedInternalButton atSetpoint = new LoggedInternalButton("Superstructure/At Setpoint");

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
                    this.pivot.setVoltage(voltage);
                    this.elevator.setLength(ElevatorConstants.minLengthPhysical);
                    this.wrist.setAngle(Degrees.zero());
                },
                (log) -> {
                    Logger.recordOutput("Superstructure/Pivot/SysID/Voltage", this.pivot.getVoltage());
                    Logger.recordOutput("Superstructure/Pivot/SysID/Position", this.pivot.getAngle());
                    Logger.recordOutput("Superstructure/Pivot/SysID/Velocity", this.pivot.getVelocity());
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
                    this.pivot.setAngle(Degrees.of(90));
                    this.elevator.setVoltage(voltage);
                    this.wrist.setAngle(Degrees.zero());
                },
                (log) -> {
                    Logger.recordOutput("Superstructure/Elevator/SysID/Voltage", this.pivot.getVoltage());
                    Logger.recordOutput("Superstructure/Elevator/SysID/Position", this.pivot.getAngle());
                    Logger.recordOutput("Superstructure/Elevator/SysID/Velocity", this.pivot.getVelocity());
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
                    this.pivot.setAngle(Degrees.of(90));
                    this.elevator.setLength(ElevatorConstants.minLengthPhysical);
                    this.wrist.setVoltage(voltage);
                },
                (log) -> {
                    Logger.recordOutput("Superstructure/Wrist/SysID/Voltage", this.wrist.getVoltage());
                    Logger.recordOutput("Superstructure/Wrist/SysID/Position", this.wrist.getAngle());
                    Logger.recordOutput("Superstructure/Wrist/SysID/Velocity", this.wrist.getVelocity());
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
        pivot.periodic();
        elevator.periodic();
        wrist.periodic();
    }

    public SuperstructureState getCurrentState() {
        return new SuperstructureState(
            pivot.getAngle(),
            elevator.getLength(),
            wrist.getAngle()
        );
    }

    public Command throttle(DoubleSupplier pivotThrottle, DoubleSupplier elevatorThrottle, DoubleSupplier wristThrottle) {
        var subsystem = this;
        return new Command() {
            private static final LoggedTunableMeasure<VoltageUnit> pivotVoltage = new LoggedTunableMeasure<>("Superstructure/Pivot Voltage", Volts.of(2));
            private static final LoggedTunableMeasure<VoltageUnit> elevatorVoltage = new LoggedTunableMeasure<>("Superstructure/Elevator Voltage", Volts.of(2));
            private static final LoggedTunableMeasure<VoltageUnit> wristVoltage = new LoggedTunableMeasure<>("Superstructure/Wrist Voltage", Volts.of(2));
            {
                addRequirements(subsystem);

            }

            @Override
            public void initialize() {
                
            }
            @Override
            public void execute() {
                pivot.setVoltage(pivotVoltage.get().times(pivotThrottle.getAsDouble()));
                elevator.setVoltage(elevatorVoltage.get().times(elevatorThrottle.getAsDouble()));
                wrist.setVoltage(wristVoltage.get().times(wristThrottle.getAsDouble()));
            }
        };
    }

    public Command idle() {
        return goToSetpointSequenced(SuperstructureState.idle);
    }

    public Command defense() {
        return goToSetpointSequenced(SuperstructureState.defense);
    }

    public Command prepareToClimb() {
        return goToSetpointSequenced(SuperstructureState.climb);
    }

    public Command climb() {
        return goToSetpointSequenced(getCurrentState());
    }

    public Command fold() {
        return goToSetpointSequenced(SuperstructureState.fold);
    }

    public Command goToSetpoint(SuperstructureState setpoint) {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("Superstructure setpoint");
            }
            @Override
            public void execute() {
                pivot.setAngle(setpoint.pivotAngle);
                elevator.setLength(setpoint.elevatorLength);
                wrist.setAngle(setpoint.wristAngle);
            }
        };
    }

    public Command goToSetpointSequenced(SuperstructureState setpoint) {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("Superstructure Setpoint Sequenced");
            }
            private SuperstructureState initialState;
            private Supplier<Angle> pivotTarget;
            private Supplier<Distance> elevatorTarget;
            private Supplier<Angle> wristTarget;
            @Override
            public void initialize() {
                this.initialState = getCurrentState();
                var extending = setpoint.elevatorLength.gt(initialState.elevatorLength);
                var elevatorMovingSignificant = !MeasureUtil.isNear(setpoint.elevatorLength, initialState.elevatorLength, Inches.of(36));
                var wristDown = setpoint.wristAngle.lt(initialState.wristAngle.minus(Degrees.of(30)));
                var pivotBackToForward = initialState.pivotAngle.gt(Degrees.of(80));
                this.pivotTarget = new Supplier<Angle>() {
                    private Angle cached = initialState.pivotAngle;
                    @Override
                    public Angle get() {
                        if (extending) {
                            if (wristDown) {
                                if (MeasureUtil.isNear(setpoint.elevatorLength, elevator.getLength(), Inches.of(5))) {
                                    cached = setpoint.pivotAngle;
                                } else {
                                    cached = Degrees.of(85);
                                }
                            } else {
                                cached = setpoint.pivotAngle;
                            }
                        } else {
                            if (MeasureUtil.isNear(setpoint.elevatorLength, elevator.getLength(), Inches.of(10))) {
                                if (pivotBackToForward) {
                                    if (MeasureUtil.isNear(setpoint.wristAngle, wrist.getAngle(), Degrees.of(3))) {
                                        cached = setpoint.pivotAngle;
                                    } else {
                                        cached = Degrees.of(60);
                                    }
                                } else {
                                    cached = setpoint.pivotAngle;
                                }
                            } else {
                                if (elevatorMovingSignificant) {
                                    cached = Degrees.of(90);
                                } else {
                                    cached = initialState.pivotAngle;
                                }
                            }
                        }
                        return cached;
                    }
                };
                this.elevatorTarget = new Supplier<Distance>() {
                    private Distance cached = initialState.elevatorLength;
                    @Override
                    public Distance get() {
                        if (extending) {
                            if (MeasureUtil.isNear(pivotTarget.get(), pivot.getAngle(), Degrees.of(5))) {
                                cached = setpoint.elevatorLength;
                            }
                        } else {
                            return setpoint.elevatorLength;
                        }
                        return cached;
                    }
                };
                this.wristTarget = new Supplier<Angle>() {
                    private Angle cached = initialState.wristAngle;
                    @Override
                    public Angle get() {
                        if (pivotBackToForward) {
                            if (MeasureUtil.isNear(pivotTarget.get(), pivot.getAngle(), Degrees.of(2))) {
                                cached = setpoint.wristAngle;
                            }
                        } else if (extending) {
                            if (wristDown) {
                                if (MeasureUtil.isNear(setpoint.elevatorLength, elevator.getLength(), Inches.of(30))) {
                                    cached = setpoint.wristAngle;
                                } else {
                                    cached = initialState.wristAngle;
                                }
                            } else {
                                cached = setpoint.wristAngle;
                            }
                        } else {
                            if (MeasureUtil.isNear(setpoint.elevatorLength, elevator.getLength(), Inches.of(5))) {
                                cached = setpoint.wristAngle;
                            } else {
                                cached = Degrees.of(10);
                            }
                        }
                        return cached;
                    }
                };
            }
            @Override
            public void execute() {
                var pivotSetpoint = pivotTarget.get();
                var elevatorSetpoint = elevatorTarget.get();
                var wristSetpoint = wristTarget.get();
                pivot.setAngle(pivotSetpoint);
                elevator.setLength(elevatorSetpoint);
                wrist.setAngle(wristSetpoint);
                Logger.recordOutput("Superstructure/Setpoint/Pivot Setpoint", pivotSetpoint);
                Logger.recordOutput("Superstructure/Setpoint/Elevator Setpoint", elevatorSetpoint);
                Logger.recordOutput("Superstructure/Setpoint/Wrist Setpoint", wristSetpoint);
                Logger.recordOutput("Superstructure/Setpoint/Mech Transforms", new SuperstructureState(pivotSetpoint, elevatorSetpoint, wristSetpoint).getMechTransforms());
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

        public static final SuperstructureState zero = new SuperstructureState(Degrees.zero(), Meters.zero(), Degrees.zero());

        public static final SuperstructureState idle = SuperstructureState.fromParts(
            Degrees.of(70),
            ElevatorConstants.minLengthPhysical,
            Degrees.of(90)
        );
        public static final SuperstructureState defense = SuperstructureState.fromParts(
            PivotConstants.minAngle,
            ElevatorConstants.minLengthPhysical,
            Degrees.of(110)
        );
        public static final SuperstructureState climb = new SuperstructureState(
            Degrees.of(100), 
            Meters.zero(),
            Degrees.of(50).minus(Degrees.of(70))
        );
        public static final SuperstructureState fold = new SuperstructureState(
            Degrees.of(-20), 
            Meters.zero(),
            Degrees.of(90).minus(Degrees.of(70))
        );
        public static SuperstructureState newConstrained(Angle pivotAngle, Distance elevatorLength, Angle wristAngle) {
            return new SuperstructureState(
                Radians.of(MathUtil.clamp(pivotAngle.in(Radians), PivotConstants.minAngle.in(Radians), PivotConstants.maxAngle.in(Radians))),
                Meters.of(MathUtil.clamp(elevatorLength.in(Meters), ElevatorConstants.minLengthPhysical.in(Meters), ElevatorConstants.maxLengthPhysical.in(Meters))),
                Radians.of(MathUtil.clamp(wristAngle.in(Radians), WristConstants.minAngle.in(Radians), WristConstants.maxAngle.in(Radians)))
            );
        }

        private SuperstructureState(Angle pivotAngle, Distance elevatorLength, Angle wristAngle) {
            this.pivotAngle = pivotAngle;
            this.elevatorLength = elevatorLength;
            this.wristAngle = wristAngle;
        }

        public static SuperstructureState fromParts(Angle pivotAngle, Distance elevatorLength, Angle wristAngle) {
            return new SuperstructureState(pivotAngle, elevatorLength, wristAngle.minus(pivotAngle));
        }

        public static SuperstructureState fromPivotSpace(Transform2d pivotSpacePose) {
            var pivotToTargetMeters = pivotSpacePose.getTranslation().getNorm();
            var elevatorPivotOffsetMeters = ElevatorConstants.pivotOffset.in(Meters);

            var pivotAngleOffset = Radians.of(Math.asin(elevatorPivotOffsetMeters / pivotToTargetMeters));

            var pivotAngle = pivotSpacePose.getTranslation().getAngle().getMeasure().minus(pivotAngleOffset);
            var wristAngle = pivotSpacePose.getRotation().minus(new Rotation2d(pivotAngle)).getMeasure();
            var elevatorHeight = Meters.of(Math.sqrt((pivotToTargetMeters * pivotToTargetMeters) - (elevatorPivotOffsetMeters * elevatorPivotOffsetMeters))).plus(ElevatorConstants.stage2Base.getMeasureX().unaryMinus());
            var elevatorLength = elevatorHeight.minus(ElevatorConstants.minHeightPhysical);

            return SuperstructureState.newConstrained(pivotAngle, elevatorLength, wristAngle);
        }

        public static SuperstructureState fromRobotSpace(Pose2d robotSpacePose) {
            return fromPivotSpace(robotSpacePose.minus(PivotConstants.pivotRobotSpace));
        }

        public Transform2d toPivotSpace() {
            var pivotRotation = new Rotation2d(pivotAngle);
            return new Transform2d(
                new Translation2d(
                    elevatorLength.plus(ElevatorConstants.minHeightPhysical).times(pivotRotation.getCos()),
                    elevatorLength.plus(ElevatorConstants.minHeightPhysical).times(pivotRotation.getSin())
                ),
                pivotRotation.plus(new Rotation2d(wristAngle))
            );
        }
        public Pose2d toRobotSpace() {
            return PivotConstants.pivotRobotSpace.plus(this.toPivotSpace());
        }

        public Transform3d[] getMechTransforms() {
            var pivotMechTransform = new Transform3d(Translation3d.kZero, new Rotation3d(Degrees.zero(), this.pivotAngle.unaryMinus(), Degrees.zero()));
            var elevatorMechTransform = new Transform3d(new Translation3d(this.elevatorLength.div(ElevatorConstants.movingStageCount), Meters.zero(), Meters.zero()), Rotation3d.kZero);
            var wristMechTransform = new Transform3d(Translation3d.kZero, new Rotation3d(Degrees.zero(), this.wristAngle.unaryMinus(), Degrees.zero()));
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

        // public SuperstructureState plus(SuperstructureState other) {
        //     return newConstrained(
        //         this.pivotAngle.plus(other.pivotAngle),
        //         this.elevatorLength.plus(other.elevatorLength),
        //         this.wristAngle.plus(other.wristAngle)
        //     );
        // }

        public boolean isNear(SuperstructureState other, Angle pivotTolerance, Distance elevatorTolerance, Angle wristTolerance) {
            return
                MeasureUtil.isNear(other.pivotAngle, this.pivotAngle, pivotTolerance) &&
                MeasureUtil.isNear(other.elevatorLength, this.elevatorLength, elevatorTolerance) &&
                MeasureUtil.isNear(other.wristAngle, this.wristAngle, wristTolerance)
            ;
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

    public static class RobotFlippedSuperstructureState {
        private final SuperstructureState forward;
        private final SuperstructureState backward;

        public RobotFlippedSuperstructureState(SuperstructureState forward, SuperstructureState backward) {
            this.forward = forward;
            this.backward = backward;
        }

        public static RobotFlippedSuperstructureState fromForwardRobotFlipped(SuperstructureState forward) {
            var robotSpacePose = forward.toRobotSpace();
            return new RobotFlippedSuperstructureState(
                forward,
                SuperstructureState.fromRobotSpace(new Pose2d(
                    new Translation2d(
                        -robotSpacePose.getX(),
                        robotSpacePose.getY()
                    ),
                    new Rotation2d(
                        -robotSpacePose.getRotation().getCos(),
                        robotSpacePose.getRotation().getSin()
                    )
                ))
            );
        }
        public static RobotFlippedSuperstructureState fromBackwardRobotFlipped(SuperstructureState backward) {
            var robotSpacePose = backward.toRobotSpace();
            return new RobotFlippedSuperstructureState(
                SuperstructureState.fromRobotSpace(new Pose2d(
                    new Translation2d(
                        -robotSpacePose.getX(),
                        robotSpacePose.getY()
                    ),
                    new Rotation2d(
                        -robotSpacePose.getRotation().getCos(),
                        robotSpacePose.getRotation().getSin()
                    )
                )),
                backward
            );
        }
        public static RobotFlippedSuperstructureState fromForwardPivotFlipped(SuperstructureState forward) {
            var pivotSpacePose = forward.toPivotSpace();
            return new RobotFlippedSuperstructureState(
                forward,
                SuperstructureState.fromPivotSpace(new Transform2d(
                    new Translation2d(
                        -pivotSpacePose.getX(),
                        pivotSpacePose.getY()
                    ),
                    new Rotation2d(
                        -pivotSpacePose.getRotation().getCos(),
                        pivotSpacePose.getRotation().getSin()
                    )
                ))
            );
        }
        public static RobotFlippedSuperstructureState fromBackwardPivotFlipped(SuperstructureState backward) {
            var pivotSpacePose = backward.toPivotSpace();
            return new RobotFlippedSuperstructureState(
                SuperstructureState.fromPivotSpace(new Transform2d(
                    new Translation2d(
                        -pivotSpacePose.getX(),
                        pivotSpacePose.getY()
                    ),
                    new Rotation2d(
                        -pivotSpacePose.getRotation().getCos(),
                        pivotSpacePose.getRotation().getSin()
                    )
                )),
                backward
            );
        }
        public static RobotFlippedSuperstructureState fromForwardOnly(SuperstructureState forward) {
            return new RobotFlippedSuperstructureState(forward, null);
        }
        public static RobotFlippedSuperstructureState fromBackwardOnly(SuperstructureState backward) {
            return new RobotFlippedSuperstructureState(null, backward);
        }

        public SuperstructureState getForward() {
            return forward;
        }
        public SuperstructureState getBackward() {
            return backward;
        }
        public SuperstructureState get(Direction direction) {
            switch (direction) {
                default:
                case Forward:   return getForward();
                case Backward:  return getBackward();
            }
        }

        public SuperstructureState getClosest(Rotation2d target, Rotation2d current) {
            if (forward == null) {
                return backward;
            } else if (backward == null) {
                return forward;
            } else {
                return get(Direction.getClosest(target, current));
            }
        }

        // public RobotFlippedSuperstructureState plus(SuperstructureState forwardOther, SuperstructureState backwardsOther) {
        //     return new RobotFlippedSuperstructureState(
        //         this.forward.plus(forwardOther),
        //         this.backward.plus(backwardsOther)
        //     );
        // }
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

    public static class RobotFlippedTotalState implements AllianceFlippable<RobotFlippedTotalState> {
        private final Pose2d forwardRobotPose;
        private final SuperstructureState forwardSuperstructureState;
        private final Pose2d backwardRobotPose;
        private final SuperstructureState backwardSuperstructureState;

        private RobotFlippedTotalState(Pose2d forwardRobotPose, SuperstructureState forwardSuperstructureState, Pose2d backwardRobotPose, SuperstructureState backwardSuperstructureState) {
            this.forwardRobotPose = forwardRobotPose;
            this.forwardSuperstructureState = forwardSuperstructureState;
            this.backwardRobotPose = backwardRobotPose;
            this.backwardSuperstructureState = backwardSuperstructureState;
        }

        public static RobotFlippedTotalState combine(RobotFlippedRobotPose robotPose, RobotFlippedSuperstructureState superstructureState) {
            Pose2d forwardRobotPose = null;
            SuperstructureState forwardSuperstructureState = null;
            Pose2d backwardRobotPose = null;
            SuperstructureState backwardSuperstructureState = null;
            if (robotPose.forward != null && superstructureState.forward != null) {
                forwardRobotPose = robotPose.getForward();
                forwardSuperstructureState = superstructureState.getForward();
            }
            if (robotPose.backward != null && superstructureState.backward != null) {
                backwardRobotPose = robotPose.getBackward();
                backwardSuperstructureState = superstructureState.getBackward();
            }
            return new RobotFlippedTotalState(forwardRobotPose, forwardSuperstructureState, backwardRobotPose, backwardSuperstructureState);
        }

        public Pose2d getRobotPose(Direction direction) {
            switch (direction) {
                default:
                case Forward:   return forwardRobotPose;
                case Backward:  return backwardRobotPose;
            }
        }
        public SuperstructureState getSuperstructureState(Direction direction) {
            switch (direction) {
                default:
                case Forward:   return forwardSuperstructureState;
                case Backward:  return backwardSuperstructureState;
            }
        }

        public Direction getClosestDirection(Rotation2d current) {
            if (forwardRobotPose == null) {
                return Direction.Backward;
            } else if (backwardRobotPose == null) {
                return Direction.Forward;
            } else {
                return Direction.getClosest(forwardRobotPose.getRotation(), current);
            }
        }

        public Pose2d getClosestRobotPose(Rotation2d current) {
            return getRobotPose(getClosestDirection(current));
        }
        public SuperstructureState getClosestSuperstructureState(Rotation2d current) {
            return getSuperstructureState(getClosestDirection(current));
        }

        @Override
        public RobotFlippedTotalState flip(FieldFlipType flipType) {
            return new RobotFlippedTotalState(
                (this.forwardRobotPose == null) ? null : AllianceFlipUtil.flip(this.forwardRobotPose, flipType),
                this.forwardSuperstructureState,
                (this.backwardRobotPose == null) ? null : AllianceFlipUtil.flip(this.backwardRobotPose, flipType),
                this.backwardSuperstructureState
            );
        }
    }
}
