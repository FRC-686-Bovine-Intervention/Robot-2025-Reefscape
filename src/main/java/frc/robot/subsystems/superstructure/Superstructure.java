package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedWeightedGraph;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
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
import frc.util.loggerUtil.tunables.LoggedTunable;

public class Superstructure extends SubsystemBase {
    public final Pivot pivot;
    public final Elevator elevator;
    public final Wrist wrist;

    private final SuperstructureState measuredState = SuperstructureState.newUnconstrained(0.0, 0.0, 0.0);

    private final Graph<SuperstructureState, Edge> graph;
    private final HashMap<SuperstructureState, HashMap<SuperstructureState, PathfindingResult[]>> graphPathfindingCache;

    private static final double edgeWeightConstant = 1;
    private static final double edgeWeightPerPivotRadians = 1;
    private static final double edgeWeightPerElevatorMeters = 1;
    private static final double edgeWeightPerWristRadians = 1;

    private SuperstructureState lastMeasuredGraphVertex;
    private SuperstructureState targetVertex;
    private Edge currentEdge;

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
                    this.pivot.setAngleGoalRadsFast(90);
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
                    this.pivot.setAngleGoalRadsFast(90);
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


        this.graph = new DefaultDirectedWeightedGraph<>(Edge.class);
        var allSuperstructureStates = new SuperstructureState[] {
            SuperstructureConstants.idleState,
            SuperstructureConstants.coralStationForwardState,
            SuperstructureConstants.coralStationBackwardState,
            SuperstructureConstants.l1State,
            SuperstructureConstants.l2State,
            SuperstructureConstants.l2l4TransferState,
            SuperstructureConstants.l3State,
            SuperstructureConstants.l3l4TransferState,
            SuperstructureConstants.l4PreState,
            SuperstructureConstants.l4State,
            SuperstructureConstants.groundAlgaeState,
            SuperstructureConstants.lowAlgaeState,
            SuperstructureConstants.lowAlgaeHoldState,
            SuperstructureConstants.highAlgaeState,
            SuperstructureConstants.highAlgaeHoldState,
            SuperstructureConstants.processorState,
            SuperstructureConstants.netForwardPreState,
            SuperstructureConstants.netForwardState,
            SuperstructureConstants.netBackwardPreState,
            SuperstructureConstants.netBackwardState,
            SuperstructureConstants.prepareClimbingState,
            SuperstructureConstants.climbingState,
            SuperstructureConstants.selfRightingState,
        };

        for (var state : allSuperstructureStates) {
            this.graph.addVertex(state);
        }


        // Graph Edges
        // | Coral Station Edges
        this.addBidirectionalEdge(SuperstructureConstants.idleState, SuperstructureConstants.coralStationForwardState);
        this.addEdge(SuperstructureConstants.idleState, SuperstructureConstants.coralStationBackwardState);
        this.addEdge(SuperstructureConstants.coralStationBackwardState, SuperstructureConstants.coralStationBackwardPulloutState);
        this.addEdge(SuperstructureConstants.coralStationBackwardPulloutState, SuperstructureConstants.idleState);
        this.addBidirectionalEdge(SuperstructureConstants.coralStationForwardState, SuperstructureConstants.coralStationBackwardState);
        // | Coral Reef Edges
        this.addBidirectionalEdge(SuperstructureConstants.idleState, SuperstructureConstants.l1State);
        this.addBidirectionalEdge(SuperstructureConstants.idleState, SuperstructureConstants.l2State);
        this.addBidirectionalEdge(SuperstructureConstants.idleState, SuperstructureConstants.l3State);
        this.addBidirectionalEdge(SuperstructureConstants.idleState, SuperstructureConstants.l4PreState);
        this.addBidirectionalEdge(SuperstructureConstants.l4PreState, SuperstructureConstants.l4State);
        // TODO Blockable edges
        // this.addEdge(SuperstructureConstants.l4State, SuperstructureConstants.idleState, true);
        this.addBidirectionalEdge(SuperstructureConstants.l2State, SuperstructureConstants.l2l4TransferState);
        this.addBidirectionalEdge(SuperstructureConstants.l3State, SuperstructureConstants.l3l4TransferState);
        this.addBidirectionalEdge(SuperstructureConstants.l2l4TransferState, SuperstructureConstants.l4PreState);
        this.addBidirectionalEdge(SuperstructureConstants.l3l4TransferState, SuperstructureConstants.l4PreState);
        // | Algae Reef Edges
        this.addBidirectionalEdge(SuperstructureConstants.idleState, SuperstructureConstants.lowAlgaeState);
        this.addBidirectionalEdge(SuperstructureConstants.idleState, SuperstructureConstants.highAlgaeState);
        this.addBidirectionalEdge(SuperstructureConstants.lowAlgaeState, SuperstructureConstants.lowAlgaeHoldState);
        this.addBidirectionalEdge(SuperstructureConstants.highAlgaeState, SuperstructureConstants.highAlgaeHoldState);
        // | Algae Net Edges
        this.addBidirectionalEdge(SuperstructureConstants.idleState, SuperstructureConstants.netForwardPreState);
        this.addBidirectionalEdge(SuperstructureConstants.idleState, SuperstructureConstants.netBackwardPreState);
        this.addBidirectionalEdge(SuperstructureConstants.netForwardPreState, SuperstructureConstants.netForwardState);
        this.addBidirectionalEdge(SuperstructureConstants.netBackwardPreState, SuperstructureConstants.netBackwardState);
        this.addBidirectionalEdge(SuperstructureConstants.netForwardState, SuperstructureConstants.netBackwardState);
        this.addBidirectionalEdge(SuperstructureConstants.lowAlgaeHoldState, SuperstructureConstants.netForwardPreState);
        this.addBidirectionalEdge(SuperstructureConstants.lowAlgaeHoldState, SuperstructureConstants.netBackwardPreState);
        this.addBidirectionalEdge(SuperstructureConstants.highAlgaeState, SuperstructureConstants.netForwardPreState);
        this.addBidirectionalEdge(SuperstructureConstants.highAlgaeState, SuperstructureConstants.netBackwardPreState);
        // | Algae Ground Edges
        this.addBidirectionalEdge(SuperstructureConstants.idleState, SuperstructureConstants.groundAlgaeState);
        this.addBidirectionalEdge(SuperstructureConstants.idleState, SuperstructureConstants.processorState);
        this.addBidirectionalEdge(SuperstructureConstants.groundAlgaeState, SuperstructureConstants.processorState);
        // | Climb Edges
        this.addBidirectionalEdge(SuperstructureConstants.idleState, SuperstructureConstants.prepareClimbingState);
        this.addBidirectionalEdge(SuperstructureConstants.idleState, SuperstructureConstants.climbingState);
        this.addBidirectionalEdge(SuperstructureConstants.prepareClimbingState, SuperstructureConstants.climbingState);
        // | Self Right Edges
        this.addBidirectionalEdge(SuperstructureConstants.idleState, SuperstructureConstants.selfRightingState);



        this.graphPathfindingCache = new HashMap<>(allSuperstructureStates.length);
        for (var fromState : allSuperstructureStates) {
            var toHashMap = new HashMap<SuperstructureState, PathfindingResult[]>(allSuperstructureStates.length);
            for (var toState : allSuperstructureStates) {
                toHashMap.put(toState, this.pathfind(fromState, toState));
            }
            this.graphPathfindingCache.put(fromState, toHashMap);
        }
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
            private static final LoggedTunable<Voltage> pivotVoltage = LoggedTunable.from("Superstructure/Pivot Voltage", Volts::of, 2);
            private static final LoggedTunable<Voltage> elevatorVoltage = LoggedTunable.from("Superstructure/Elevator Voltage", Volts::of, 2);
            private static final LoggedTunable<Voltage> wristVoltage = LoggedTunable.from("Superstructure/Wrist Voltage", Volts::of, 2);
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

    public Command directToState(SuperstructureState goalState) {
        final var superstructure = this;
        return new Command() {
            {
                this.addRequirements(superstructure);
                this.setName("Direct to State");
            }
            @Override
            public void execute() {
                superstructure.pivot.setAngleGoalRadsFast(goalState.getPivotAngleRads());
                superstructure.elevator.setLengthGoalMeters(goalState.getElevatorLengthMeters());
                superstructure.wrist.setAngleGoalRads(goalState.getWristAngleRads());
            }
            @Override
            public void end(boolean interrupted) {
                superstructure.pivot.stop(NeutralMode.DEFAULT);
                superstructure.elevator.stop(NeutralMode.DEFAULT);
                superstructure.wrist.stop(NeutralMode.DEFAULT);
            }
        };
    }

    public Command goToStatePathfinded(SuperstructureState goalState) {
        final var superstructure = this;
        return new Command() {
            {
                this.addRequirements(superstructure);
                this.setName("Go To State Pathfinded");
            }

            private PathfindingResult commandPath;
            private int currentEdgeIndex;

            @Override
            public void initialize() {
                if (superstructure.targetVertex != superstructure.lastMeasuredGraphVertex) {
                    var fromLastVertexPathfindingResult = superstructure.pathfindFromCache(superstructure.lastMeasuredGraphVertex, goalState);
                    var fromTargetPathfindingResult = superstructure.pathfindFromCache(superstructure.targetVertex, goalState);
                    if (fromLastVertexPathfindingResult.get().totalWeight() < fromTargetPathfindingResult.get().totalWeight()) {
                        // Cancel previous command and return to previous state
                        this.commandPath = fromLastVertexPathfindingResult.get();
                        superstructure.targetVertex = superstructure.lastMeasuredGraphVertex;
                        superstructure.currentEdge = null;
                    } else {
                        // Continuing previous command
                        this.commandPath = fromTargetPathfindingResult.get();
                    }
                    this.currentEdgeIndex = -1;
                } else {
                    this.commandPath = superstructure.pathfind(superstructure.targetVertex, goalState).get();
                    if (this.commandPath.edgePath().length > 0) {
                        superstructure.currentEdge = this.commandPath.edgePath()[0];
                    } else {
                        superstructure.currentEdge = null;
                    }
                    this.currentEdgeIndex = 0;
                }
                if (superstructure.currentEdge != null) {
                    // superstructure.currentEdge.initialize();
                }
            }

            @Override
            public void execute() {
                var firstLoop = true;
                while (firstLoop || (superstructure.currentEdge == null) ? (superstructure.lastMeasuredGraphVertex == superstructure.targetVertex) : (superstructure.currentEdge.isFinished())) {
                    if (superstructure.currentEdge == null) {
                        if (superstructure.measuredState.isNear(superstructure.targetVertex,
                            Units.degreesToRadians(1.0),
                            Units.inchesToMeters(1.0),
                            Units.degreesToRadians(1.0)
                        )) {
                            superstructure.lastMeasuredGraphVertex = superstructure.targetVertex;
                        } else {
                            superstructure.pivot.setAngleGoalRadsFast(superstructure.targetVertex.getPivotAngleRads());
                            superstructure.elevator.setLengthGoalMeters(superstructure.targetVertex.getElevatorLengthMeters());
                            superstructure.wrist.setAngleGoalRads(superstructure.targetVertex.getWristAngleRads());
                        }
                    } else {
                        // superstructure.currentEdge.execute();
                    }
                    firstLoop = false;
                }
            }
            
            @Override
            public void end(boolean interrupted) {
                if (superstructure.currentEdge != null) {
                    // superstructure.currentEdge.end(interrupted);
                }
                superstructure.pivot.stop(NeutralMode.DEFAULT);
                superstructure.elevator.stop(NeutralMode.DEFAULT);
                superstructure.wrist.stop(NeutralMode.DEFAULT);
            }
        };
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

    private void addEdge(SuperstructureState fromState, SuperstructureState toState) {
        this.graph.addEdge(fromState, toState, new Edge(calculateDefaultEdgeWeight(fromState, toState), false, this.directToState(toState)));
    }

    private void addBidirectionalEdge(SuperstructureState fromState, SuperstructureState toState) {
        this.addEdge(fromState, toState);
        this.addEdge(toState, fromState);
    }

    private PathfindingResult[] pathfindFromCache(SuperstructureState fromState, SuperstructureState toState) {
        var toHashMap = this.graphPathfindingCache.get(fromState);
        if (toHashMap == null) {
            return new PathfindingResult[0];
        }
        var pathfindingResult = toHashMap.get(toState);
        if (pathfindingResult == null) {
            return new PathfindingResult[0];
        }
        return pathfindingResult;
    }

    private PathfindingResult[] pathfind(SuperstructureState fromState, SuperstructureState toState) {


        return new PathfindingResult[0];
    }

    private static record PathfindingResult(
        double totalWeight,
        Command[] edgePath
    ) {

    }

    private static record Edge(
        double weight,
        boolean blockable,
        Command command
    ) {

    }

    private static double calculateDefaultEdgeWeight(SuperstructureState a, SuperstructureState b) {
        var pivotAngleDiffRads = Math.abs(a.getPivotAngleRads() - b.getPivotAngleRads());
        var elevatorLengthDiffMeters = Math.abs(a.getElevatorLengthMeters() - b.getElevatorLengthMeters());
        var wristAngleDiffRads = Math.abs(a.getWristAngleRads() - b.getWristAngleRads());

        return
            edgeWeightConstant
            + pivotAngleDiffRads * edgeWeightPerPivotRadians
            + elevatorLengthDiffMeters * edgeWeightPerElevatorMeters
            + wristAngleDiffRads * edgeWeightPerWristRadians
        ;
    }
}
