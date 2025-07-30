package frc.robot.subsystems.objectiveTracker;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Coral;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Reef.BranchLevel;
import frc.robot.constants.FieldConstants.Reef.BranchObject;
import frc.robot.constants.FieldConstants.Reef.PipeObject;
import frc.robot.constants.FieldConstants.Reef.RackObject;
import frc.robot.constants.FieldConstants.Reef.StagedAlgaeLevel;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.objectiveTracker.objectives.ClimbObjective;
import frc.robot.subsystems.objectiveTracker.objectives.IntakeAlgaeObjective;
import frc.robot.subsystems.objectiveTracker.objectives.IntakeCoralObjective;
import frc.robot.subsystems.objectiveTracker.objectives.Objective;
import frc.robot.subsystems.objectiveTracker.objectives.Objective.ObjectiveType;
import frc.robot.subsystems.objectiveTracker.objectives.ScoreAlgaeObjective;
import frc.robot.subsystems.objectiveTracker.objectives.ScoreCoralObjective;
import frc.robot.subsystems.superstructure.Superstructure.Direction;
import frc.robot.subsystems.superstructure.Superstructure.RobotFlippedRobotPose;
import frc.util.VirtualSubsystem;
import frc.util.flipping.AllianceFlipped;
import frc.util.loggerUtil.LoggerUtil;

public class ObjectiveTracker extends VirtualSubsystem {
    private final ReefTrackerIO io;
    private final ReefTrackerIOInputsAutoLogged inputs = new ReefTrackerIOInputsAutoLogged();

    public static enum AlgaeGoal {
        NET,
        NET_OPPONENT_SIDE,
        PROCESSOR,
        OPPONENT_PROCESSOR,
        ;
    }

    public static enum CoralGoal {
        BRANCH,
        LEVEL1
        ;
    }

    public static enum Mode {
        Smart,
        Dumb,
    }

    public static enum Priority {
        Level4Fill(Optional.of(BranchLevel.Level4), false) {
            @Override
            public boolean isCompleted(boolean[] branchStates, int level1Count) {
                for (int i = 24; i < 36; i++) {
                    if (branchStates[i] == false) return false;
                }
                return true;
            }
        },
        Level3Fill(Optional.of(BranchLevel.Level3), false) {
            @Override
            public boolean isCompleted(boolean[] branchStates, int level1Count) {
                for (int i = 12; i < 24; i++) {
                    if (branchStates[i] == false) return false;
                }
                return true;
            }
        },
        Level2Fill(Optional.of(BranchLevel.Level2), false) {
            @Override
            public boolean isCompleted(boolean[] branchStates, int level1Count) {
                for (int i = 0; i < 12; i++) {
                    if (branchStates[i] == false) return false;
                }
                return true;
            }
        },
        Level1Fill(Optional.empty(), false) {
            @Override
            public boolean isCompleted(boolean[] branchStates, int level1Count) {
                return false;
            }
        },
        Level4RP(Optional.of(BranchLevel.Level4), true) {
            @Override
            public boolean isCompleted(boolean[] branchStates, int level1Count) {
                var count = 0;
                for (int i = 24; i < 36; i++) {
                    if (branchStates[i] == true) count++;
                    if (count >= 7) return true;
                }
                return false;
            }
        },
        Level3RP(Optional.of(BranchLevel.Level3), true) {
            @Override
            public boolean isCompleted(boolean[] branchStates, int level1Count) {
                var count = 0;
                for (int i = 12; i < 24; i++) {
                    if (branchStates[i] == true) count++;
                    if (count >= 7) return true;
                }
                return false;
            }
        },
        Level2RP(Optional.of(BranchLevel.Level2), true) {
            @Override
            public boolean isCompleted(boolean[] branchStates, int level1Count) {
                var count = 0;
                for (int i = 0; i < 12; i++) {
                    if (branchStates[i] == true) count++;
                    if (count >= 7) return true;
                }
                return false;
            }
        },
        Level1RP(Optional.empty(), true) {
            @Override
            public boolean isCompleted(boolean[] branchStates, int level1Count) {
                return level1Count >= 7;
            }
        },
        ;
        public final Optional<BranchLevel> level;
        public final boolean isRP;
        Priority(Optional<BranchLevel> level, boolean isRP) {
            this.level = level;
            this.isRP = isRP;
        }
        public boolean isCompleted(boolean[] branchStates, int level1Count) {
            return false;
        }
    }

    private AlgaeGoal selectedAlgaeGoal = AlgaeGoal.NET;
    private Pair<CoralGoal, Integer> selectedCoralGoal = new Pair<>(CoralGoal.BRANCH, 0);

    private Mode mode = Mode.Dumb;

    private final boolean[] branchStates = new boolean[] {
        false,false,false,false,false,false,false,false,false,false,false,false,
        false,false,false,false,false,false,false,false,false,false,false,false,
        false,false,false,false,false,false,false,false,false,false,false,false,
    };
    private int level1Count = 0;
    private final boolean[] algaeStates = new boolean[] {true,true,true,true,true,true};
    private boolean coopState = false;

    private final Set<ScoreCoralObjective> allScoreCoralObjectives;
    private final Set<ScoreCoralObjective> availableScoreCoralObjectives = new HashSet<>(48);
    private final Set<ScoreCoralObjective> availableUnblockedScoreCoralObjectives = new HashSet<>(48);

    private final Set<ScoreAlgaeObjective> allScoreAlgaeObjectives;
    
    private final Set<IntakeCoralObjective> allIntakeCoralObjectives;
    
    private final Set<IntakeAlgaeObjective> allIntakeAlgaeObjectives;
    private final Set<IntakeAlgaeObjective> availableIntakeAlgaeObjectives = new HashSet<>(12);

    private final Set<ClimbObjective> allClimbObjectives;

    private final List<Priority> fullStrategy = new ArrayList<>(List.of(
        Priority.Level4Fill,
        Priority.Level3Fill,
        Priority.Level2Fill,
        Priority.Level1Fill,
        Priority.Level4RP,
        Priority.Level3RP,
        Priority.Level2RP,
        Priority.Level1RP
    ));
    private final List<Priority> uncompletedPriorities = new ArrayList<>(fullStrategy.size());

    private IntakeCoralObjective intakeCoralObjective;
    private Optional<IntakeAlgaeObjective> intakeAlgaeObjective;
    private ScoreCoralObjective scoreCoralObjective;
    private ScoreAlgaeObjective scoreAlgaeObjective;
    private ClimbObjective climbObjective;
    private Optional<Objective> target = Optional.empty();
    private Optional<ObjectiveType> typeOverride = Optional.empty();
    private Optional<Optional<BranchLevel>> levelLock = Optional.empty();
    private Optional<PipeObject> pipeLock = Optional.empty();

    public ObjectiveTracker(ReefTrackerIO io) {
        System.out.println("[Init ObjectiveTracker] Instantiating ObjectiveTracker with " + io.getClass().getSimpleName());
        this.io = io;

        var scoreCoralObjectives = new ScoreCoralObjective[(FieldConstants.Reef.racks.length * 2) + FieldConstants.Reef.branches.length];
        var leftL1Transform = new Transform2d();
        var rightL1Transform = new Transform2d();
        for (var rackConcept : FieldConstants.Reef.racks) {
            scoreCoralObjectives[(rackConcept.id * 2) + 0] = new ScoreCoralObjective(rackConcept.map((rackObject) -> rackObject.centerRobotPose.getForward().transformBy(leftL1Transform)), Optional.empty(), Direction.Forward);
            scoreCoralObjectives[(rackConcept.id * 2) + 1] = new ScoreCoralObjective(rackConcept.map((rackObject) -> rackObject.centerRobotPose.getForward().transformBy(rightL1Transform)), Optional.empty(), Direction.Forward);
        }
        for (var branchConcept : FieldConstants.Reef.branches) {
            scoreCoralObjectives[12 + branchConcept.id] = new ScoreCoralObjective(branchConcept.map((branchObject) -> branchObject.scoreTotalState.getRobotPose(Direction.Forward)), Optional.of(branchConcept), Direction.Forward);
        }
        this.allScoreCoralObjectives = Set.of(scoreCoralObjectives);

        this.allScoreAlgaeObjectives = Set.of(
            new ScoreAlgaeObjective(FieldConstants.Barge.frontLeftBargePose.map((barge) -> barge.get(Direction.Forward)), false, Direction.Forward),
            new ScoreAlgaeObjective(FieldConstants.Barge.frontLeftBargePose.map((barge) -> barge.get(Direction.Forward)), false, Direction.Forward),
            new ScoreAlgaeObjective(FieldConstants.Barge.frontLeftBargePose.map((barge) -> barge.get(Direction.Forward)), false, Direction.Forward),
            new ScoreAlgaeObjective(FieldConstants.Barge.frontLeftBargePose.map((barge) -> barge.get(Direction.Forward)), false, Direction.Forward),
            new ScoreAlgaeObjective(FieldConstants.Barge.frontLeftBargePose.map((barge) -> barge.get(Direction.Forward)), false, Direction.Forward),
            new ScoreAlgaeObjective(FieldConstants.Barge.frontLeftBargePose.map((barge) -> barge.get(Direction.Forward)), false, Direction.Forward),
            new ScoreAlgaeObjective(FieldConstants.Barge.frontLeftBargePose.map((barge) -> barge.get(Direction.Forward)), false, Direction.Forward),
            new ScoreAlgaeObjective(FieldConstants.Barge.frontLeftBargePose.map((barge) -> barge.get(Direction.Forward)), false, Direction.Forward),
            new ScoreAlgaeObjective(FieldConstants.Barge.frontLeftBargePose.map((barge) -> barge.get(Direction.Forward)), false, Direction.Forward),
            new ScoreAlgaeObjective(FieldConstants.Barge.frontLeftBargePose.map((barge) -> barge.get(Direction.Forward)), false, Direction.Forward),
            new ScoreAlgaeObjective(FieldConstants.Barge.frontLeftBargePose.map((barge) -> barge.get(Direction.Forward)), false, Direction.Forward),
            new ScoreAlgaeObjective(FieldConstants.Barge.frontLeftBargePose.map((barge) -> barge.get(Direction.Forward)), false, Direction.Forward),
            new ScoreAlgaeObjective(FieldConstants.Processor.processorTargetPose.map((barge) -> barge.get(Direction.Forward)), true, Direction.Forward),
            new ScoreAlgaeObjective(FieldConstants.Processor.processorTargetPose.invert().map((barge) -> barge.get(Direction.Forward)), true, Direction.Forward)
        );

        this.allIntakeCoralObjectives = Set.of(
            new IntakeCoralObjective(FieldConstants.CoralStation.leftStationLeft.map((station) -> station.get(Direction.Forward)), Direction.Forward),
            new IntakeCoralObjective(FieldConstants.CoralStation.leftStationCenter.map((station) -> station.get(Direction.Forward)), Direction.Forward),
            new IntakeCoralObjective(FieldConstants.CoralStation.leftStationRight.map((station) -> station.get(Direction.Forward)), Direction.Forward),
            new IntakeCoralObjective(FieldConstants.CoralStation.leftStationLeft.map((station) -> station.get(Direction.Backward)), Direction.Backward),
            new IntakeCoralObjective(FieldConstants.CoralStation.leftStationCenter.map((station) -> station.get(Direction.Backward)), Direction.Backward),
            new IntakeCoralObjective(FieldConstants.CoralStation.leftStationRight.map((station) -> station.get(Direction.Backward)), Direction.Backward),
            new IntakeCoralObjective(FieldConstants.CoralStation.rightStationLeft.map((station) -> station.get(Direction.Forward)), Direction.Forward),
            new IntakeCoralObjective(FieldConstants.CoralStation.rightStationCenter.map((station) -> station.get(Direction.Forward)), Direction.Forward),
            new IntakeCoralObjective(FieldConstants.CoralStation.rightStationRight.map((station) -> station.get(Direction.Forward)), Direction.Forward),
            new IntakeCoralObjective(FieldConstants.CoralStation.rightStationLeft.map((station) -> station.get(Direction.Backward)), Direction.Backward),
            new IntakeCoralObjective(FieldConstants.CoralStation.rightStationCenter.map((station) -> station.get(Direction.Backward)), Direction.Backward),
            new IntakeCoralObjective(FieldConstants.CoralStation.rightStationRight.map((station) -> station.get(Direction.Backward)), Direction.Backward)
        );

        var intakeAlgaeObjectives = new IntakeAlgaeObjective[FieldConstants.Reef.stagedAlgae.length + FieldConstants.Reef.stagedAlgae.length];
        for (var algaeConcept : FieldConstants.Reef.stagedAlgae) {
            intakeAlgaeObjectives[algaeConcept.rack.id] = new IntakeAlgaeObjective(algaeConcept.map((algaeObject) -> algaeObject.intakeTotalState.getRobotPose(Direction.Forward)), algaeConcept, Direction.Forward);
        }
        for (var algaeConcept : FieldConstants.Reef.stagedAlgae) {
            intakeAlgaeObjectives[FieldConstants.Reef.stagedAlgae.length + algaeConcept.rack.id] = new IntakeAlgaeObjective(algaeConcept.map((algaeObject) -> algaeObject.intakeTotalState.getRobotPose(Direction.Forward)), algaeConcept, Direction.Forward);
        }
        this.allIntakeAlgaeObjectives = Set.of(intakeAlgaeObjectives);

        this.allClimbObjectives = Set.of(
            new ClimbObjective(FieldConstants.Barge.leftCagePose.map((cage) -> cage.get(Direction.Forward))),
            new ClimbObjective(FieldConstants.Barge.centerCagePose.map((cage) -> cage.get(Direction.Forward))),
            new ClimbObjective(FieldConstants.Barge.rightCagePose.map((cage) -> cage.get(Direction.Forward))),
            new ClimbObjective(FieldConstants.Barge.leftCagePose.map((cage) -> cage.get(Direction.Backward))),
            new ClimbObjective(FieldConstants.Barge.centerCagePose.map((cage) -> cage.get(Direction.Backward))),
            new ClimbObjective(FieldConstants.Barge.rightCagePose.map((cage) -> cage.get(Direction.Backward)))
        );

        this.updateAvailableScoreCoralObjectives();
        this.updateAvailableIntakeAlgaeObjectives();
        this.updateUnblockedScoreCoralObjectives();
        this.updateIncompletePriorities();
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Inputs/Objective Tracker", this.inputs);

        if (this.inputs.mode != -1) {
            this.mode = Mode.values()[this.inputs.mode];
            this.inputs.mode = -1;
        }
        if (this.inputs.coralGoal != -1) {
            if (this.inputs.coralGoal >= 36) {
                this.selectedCoralGoal = new Pair<>(CoralGoal.LEVEL1, this.inputs.coralGoal - 36);
            } else {
                this.selectedCoralGoal = new Pair<>(CoralGoal.BRANCH, this.inputs.coralGoal);
            }
            this.inputs.coralGoal = -1;
        }
        if (this.inputs.algaeGoal != -1) {
            this.selectedAlgaeGoal = AlgaeGoal.values()[this.inputs.algaeGoal];
            this.inputs.algaeGoal = -1;
        }

        this.io.setMode(this.mode.ordinal());
        switch (this.selectedCoralGoal.getFirst()) {
            case LEVEL1:
                this.io.setCoralGoal(this.selectedCoralGoal.getSecond() + 36);
                break;
            case BRANCH:
                this.io.setCoralGoal(this.selectedCoralGoal.getSecond());
                break;
        }
        this.io.setAlgaeGoal(this.selectedAlgaeGoal.ordinal());

        var branchesChanged = false;
        for (var changedBranch : this.inputs.branchQueue) {
            branchesChanged = true;
            var branchState = changedBranch >= 0;
            var branchID = branchState ? changedBranch : changedBranch + 36;
            this.branchStates[(int) branchID] = branchState;
        }

        var level1Changed = false;
        for (var changedLevel1 : inputs.level1Queue) {
            level1Changed = true;
            level1Count += changedLevel1;
        }

        var algaeChanged = false;
        for (var changedBranch : inputs.algaeQueue) {
            algaeChanged = true;
            var algaeState = changedBranch >= 0;
            var algaeID = algaeState ? changedBranch : changedBranch + 6;
            this.algaeStates[(int) algaeID] = algaeState;
        }

        var strategyChanged = false;
        for (var changedPriority : this.inputs.priorityListQueue) {
            strategyChanged = true;
            var oldIndex = changedPriority[0];
            var newIndex = changedPriority[1];
            Collections.swap(this.fullStrategy, oldIndex, newIndex);
        }

        var coopChanged = false;
        for (var changedCoop : this.inputs.coop) {
            coopChanged = true;
            this.coopState = changedCoop;
        }

        if (branchesChanged) {
            this.updateAvailableScoreCoralObjectives();
        }

        if (algaeChanged) {
            this.updateAvailableIntakeAlgaeObjectives();
        }

        if (algaeChanged || branchesChanged) {
            this.updateUnblockedScoreCoralObjectives();
        }

        if (branchesChanged || strategyChanged || level1Changed || coopChanged) {
            this.updateIncompletePriorities();
        }

        // TODO: ON ALLIANCE CHANGE UPDATE ALL

        for (var priority : this.fullStrategy) {
            Logger.recordOutput(
                switch (priority) {
                    case Level4Fill -> "Objective Tracker/Priorities/Fill/Level 4";
                    case Level3Fill -> "Objective Tracker/Priorities/Fill/Level 3";
                    case Level2Fill -> "Objective Tracker/Priorities/Fill/Level 2";
                    case Level1Fill -> "Objective Tracker/Priorities/Fill/Level 1";
                    case Level4RP -> "Objective Tracker/Priorities/RP/Level 4";
                    case Level3RP -> "Objective Tracker/Priorities/RP/Level 3";
                    case Level2RP -> "Objective Tracker/Priorities/RP/Level 2";
                    case Level1RP -> "Objective Tracker/Priorities/RP/Level 1";
                },
                priority.isCompleted(this.branchStates, this.level1Count)
            );
        }

        this.io.setCoralState(this.branchStates);
        this.io.setLevel1Count(this.level1Count);
        this.io.setAlgaeState(this.algaeStates);
        this.io.setCoopState(this.coopState);
        this.io.setPriorityList(this.fullStrategy.stream().mapToInt(Enum::ordinal).toArray());

        Logger.recordOutput("Objective Tracker/Priorities/Strategy/Full", this.fullStrategy.toArray(Priority[]::new));
        Logger.recordOutput("Objective Tracker/Priorities/Strategy/Uncomplete", this.uncompletedPriorities.toArray(Priority[]::new));
    }

    private void updateAvailableScoreCoralObjectives() {
        this.availableScoreCoralObjectives.clear();
        for (var objective : this.allScoreCoralObjectives) {
            if (objective.getTargetBranch().isEmpty() || this.branchStates[objective.getTargetBranch().get().id] == false) {
                this.availableScoreCoralObjectives.add(objective);
            }
        }
        var filledBranches = new ArrayList<Pose3d>(36);
        for (int i = 0; i < this.branchStates.length; i++) {
            if (this.branchStates[i] == true) {
                filledBranches.add(Reef.reefs.getOurs().branches[i].branchTip.transformBy(Coral.branchPlacement));
            }
        }
        Logger.recordOutput("Objective Tracker/Reef/Coral", filledBranches.toArray(Pose3d[]::new));
    }
    private void updateAvailableIntakeAlgaeObjectives() {
        this.availableIntakeAlgaeObjectives.clear();
        for (var objective : this.allIntakeAlgaeObjectives) {
            if (this.algaeStates[objective.getTargetAlgae().rack.id] == false) {
                this.availableIntakeAlgaeObjectives.add(objective);
            }
        }
        var filledAlgae = new ArrayList<Pose3d>(6);
        for (int i = 0; i < this.algaeStates.length; i++) {
            if (this.algaeStates[i] == true) {
                filledAlgae.add(Reef.reefs.getOurs().stagedAlgae[i].centerPose);
            }
        }
        Logger.recordOutput("Objective Tracker/Reef/Algae", filledAlgae.toArray(Pose3d[]::new));
    }
    private void updateUnblockedScoreCoralObjectives() {
        this.availableUnblockedScoreCoralObjectives.clear();
        for (var objective : this.availableScoreCoralObjectives) {
            if (
                objective.getTargetBranch().isEmpty()
                || objective.getTargetBranch().get().level == BranchLevel.Level4
                || this.algaeStates[objective.getTargetBranch().get().pipe.rack.id] == false
                || (
                    objective.getTargetBranch().get().pipe.rack.stagedAlgae.level == StagedAlgaeLevel.High
                    && objective.getTargetBranch().get().level == BranchLevel.Level2
                )
            ) {
                this.availableUnblockedScoreCoralObjectives.add(objective);
            }
        }
    }
    public void updateIncompletePriorities() {
        this.uncompletedPriorities.clear();
        this.uncompletedPriorities.addAll(this.fullStrategy);
        this.uncompletedPriorities.removeIf((priority) -> priority.isCompleted(this.branchStates, this.level1Count));
        if (this.coopState == true) {
            for (int i = this.uncompletedPriorities.size() - 1; i >= 0; i--) {
                var priority = this.uncompletedPriorities.get(i);
                if (priority.isRP) {
                    this.uncompletedPriorities.remove(i);
                    break;
                }
            }
        }
    }

    public void determineGoal(Pose2d currentPose, boolean hasCoral, boolean hasAlgae) {
        // var stationPoses = new RobotFlippedRobotPose[] {
        //     CoralStation.leftStationLeft.getOurs(),
        //     CoralStation.leftStationCenter.getOurs(),
        //     CoralStation.leftStationRight.getOurs(),
        //     CoralStation.rightStationLeft.getOurs(),
        //     CoralStation.rightStationCenter.getOurs(),
        //     CoralStation.rightStationRight.getOurs(),
        // };
        // var closestStationPose = Arrays.stream(stationPoses).sorted((a,b) -> {
        //     var aDistance = a.getClosest(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
        //     var bDistance = b.getClosest(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
        //     return (int) Math.signum(aDistance - bDistance);
        // }).findFirst().get();
        // this.intakeCoralObjective = new IntakeCoralObjective(closestStationPose, currentPose.getRotation());
        this.intakeCoralObjective = this.allIntakeCoralObjectives
            .stream()
            .sorted(Comparator.comparingDouble((objective) -> currentPose.getTranslation().getDistance(objective.getTargetPose().getOurs().getTranslation())))
            .findFirst()
            .get()
        ;

        // var closestRacks = Arrays.stream(Reef.reefs.getOurs().racks)
        //     .sorted((a,b) -> {
        //         var aDistance = a.centerRobotPose.getClosest(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
        //         var bDistance = b.centerRobotPose.getClosest(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
        //         return (int) Math.signum(aDistance - bDistance);
        //     })
        //     .limit(3)
        //     .toList()
        // ;

        // var closestAlgae = availableAlgae.stream().map((algae) -> algae.getOurs())
        //     .filter((algae) -> closestRacks.contains(algae.rack))
        //     .sorted((a,b) -> {
        //         var aDistance = a.intakeTotalState.getClosestRobotPose(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
        //         var bDistance = b.intakeTotalState.getClosestRobotPose(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
        //         return (int) Math.signum(aDistance - bDistance);
        //     })
        //     .findFirst()
        // ;
        // this.intakeAlgaeObjective = closestAlgae.map((algae) -> new IntakeAlgaeObjective(algae, currentPose.getRotation()));
        this.intakeAlgaeObjective = this.allIntakeAlgaeObjectives
            .stream()
            .sorted(Comparator.comparingDouble((objective) -> currentPose.getTranslation().getDistance(objective.getTargetPose().getOurs().getTranslation())))
            .limit(3)
            .filter(this.availableIntakeAlgaeObjectives::contains)
            .findFirst()
        ;

        if (mode == Mode.Smart) {
            var closestPipes = Arrays.stream(Reef.reefs.getOurs().pipes)
                .sorted(Comparator.comparingDouble((pipe) -> currentPose.getTranslation().getDistance(pipe.robotPose.getForward().getTranslation())))
                .limit(6)
                .toList()
            ;
            // var targetBranch = availableUnblockedBranches.stream()
            //     .map((branch) -> branch.getOurs())
            //     .filter((branch) -> closestPipes.contains(branch.pipe))
            //     .filter((branch) -> levelLock.isEmpty() || (levelLock.get().isPresent() && branch.level == levelLock.get().get()))
            //     .filter((branch) -> pipeLock.isEmpty() || (branch.pipe == pipeLock.get()))
            //     .sorted((a,b) -> {
            //         if (a.level == b.level) {
            //             var aDistance = a.scoreTotalState.getClosestRobotPose(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
            //             var bDistance = b.scoreTotalState.getClosestRobotPose(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
            //             return (int) Math.signum(aDistance - bDistance);
            //         } else {
            //             for (var priority : uncompletedPriorities) {
            //                 if (priority.level.isEmpty()) continue;
            //                 if (a.level == priority.level.get()) return -1;
            //                 if (b.level == priority.level.get()) return 1;
            //             }
            //             return 0;
            //         }
            //     })
            //     .findFirst()
            // ;

            var target = this.availableUnblockedScoreCoralObjectives
                .stream()
                .filter((branchOrLevel1) -> branchOrLevel1.getTargetBranch().isEmpty() || closestPipes.contains(branchOrLevel1.getTargetBranch().get().getOurs().pipe))
                .filter((branchOrLevel1) -> levelLock.isEmpty() || (branchOrLevel1.getTargetBranch().map((branch) -> branch.level).equals(levelLock.get())) || (pipeLock.isPresent() && (Arrays.stream(pipeLock.get().branches).anyMatch((branch) -> branchStates[branch.id] == false))))
                .filter((branchOrLevel1) -> pipeLock.isEmpty() || (branchOrLevel1.getTargetBranch().isPresent() && branchOrLevel1.getTargetBranch().get().getOurs().pipe == pipeLock.get()) || (Arrays.stream(pipeLock.get().branches).allMatch((branch) -> branchStates[branch.id] == true)))
                .sorted((a,b) -> {
                    if (a.getTargetBranch().equals(b.getTargetBranch())) {
                        return Double.compare(
                            currentPose.getTranslation().getDistance(a.getTargetPose().getOurs().getTranslation()),
                            currentPose.getTranslation().getDistance(b.getTargetPose().getOurs().getTranslation())
                        );
                    } else {
                        for (var priority : uncompletedPriorities) {
                            if (a.getTargetBranch().map((branch) -> branch.level).equals(priority.level)) return -1;
                            if (b.getTargetBranch().map((branch) -> branch.level).equals(priority.level)) return 1;
                        }
                        return 0;
                    }
                })
                .findFirst()
            ;
            // if (uncompletedPriorities.get(0).level.isEmpty() || targetBranch.isEmpty()) {
            //     this.scoreCoralObjective = ScoreCoralObjective.fromLevel1(closestRacks.get(0), currentPose.getRotation());
            // } else {
            //     this.scoreCoralObjective = ScoreCoralObjective.fromBranch(targetBranch.get(), currentPose.getRotation());
            // }
            if (target.isPresent()) {
                this.scoreCoralObjective = target.get();
            }

            Leds.getInstance().goToOppositeSideOfReef.setFlag(false);
            Leds.getInstance().removeAlgae.setFlag(false);
            if (!this.scoreCoralObjective.getTargetBranch().map((branch) -> branch.level).equals(uncompletedPriorities.get(0).level)) {
                var topPriorityLevel = uncompletedPriorities.get(0).level.get();
                if (closestPipes.stream().allMatch((pipe) -> branchStates[topPriorityLevel.ordinal() * 12 + pipe.id] == true)) {
                    Leds.getInstance().goToOppositeSideOfReef.setFlag(true);
                } else {
                    Leds.getInstance().removeAlgae.setFlag(true);
                }
            }
        } else {
            this.scoreCoralObjective = switch (selectedCoralGoal.getFirst()) {
                case BRANCH -> this.scoreCoralObjective = this.allScoreCoralObjectives
                    .stream()
                    .filter((objective) -> objective.getTargetBranch().isPresent() && objective.getTargetBranch().get().id == this.selectedCoralGoal.getSecond().intValue())
                    .sorted(Comparator.comparingDouble((objective) -> currentPose.getTranslation().getDistance(objective.getTargetPose().getOurs().getTranslation())))
                    .findFirst()
                    .get()
                ;
                case LEVEL1 -> this.scoreCoralObjective = this.allScoreCoralObjectives
                    .stream()
                    .filter((objective) -> objective.getTargetBranch().isEmpty() && objective.getTargetBranch().get().id == this.selectedCoralGoal.getSecond().intValue())
                    .sorted(Comparator.comparingDouble((objective) -> currentPose.getTranslation().getDistance(objective.getTargetPose().getOurs().getTranslation())))
                    .findFirst()
                    .get()
                ;
            };
        }

        Leds.getInstance().level1Targeted.setFlag(this.scoreCoralObjective.getTargetBranch().isEmpty());
        Leds.getInstance().level2Targeted.setFlag(this.scoreCoralObjective.getTargetBranch().equals(Optional.of(BranchLevel.Level2)));
        Leds.getInstance().level3Targeted.setFlag(this.scoreCoralObjective.getTargetBranch().equals(Optional.of(BranchLevel.Level3)));
        Leds.getInstance().level4Targeted.setFlag(this.scoreCoralObjective.getTargetBranch().equals(Optional.of(BranchLevel.Level4)));

        this.scoreAlgaeObjective = this.allScoreAlgaeObjectives
            .stream()
            .sorted(Comparator.comparingDouble((objective) -> currentPose.getTranslation().getDistance(objective.getTargetPose().getOurs().getTranslation())))
            .findFirst()
            .get()
        ;

        // var cagePoses = new RobotFlippedRobotPose[] {
        //     Barge.leftCagePose.getOurs(),
        //     Barge.centerCagePose.getOurs(),
        //     Barge.rightCagePose.getOurs()
        // };
        // var closestCagePose = Arrays.stream(cagePoses).sorted((a,b) -> {
        //     var aDistance = a.getClosest(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
        //     var bDistance = b.getClosest(currentPose.getRotation()).getTranslation().getDistance(currentPose.getTranslation());
        //     return (int) Math.signum(aDistance - bDistance);
        // }).findFirst().get();

        // this.climbObjective = new ClimbObjective(closestCagePose, currentPose.getRotation());
        this.climbObjective = this.allClimbObjectives
            .stream()
            .sorted(Comparator.comparingDouble((objective) -> currentPose.getTranslation().getDistance(objective.getTargetPose().getOurs().getTranslation())))
            .findFirst()
            .get()
        ;

        Logger.recordOutput("Objective Tracker/Intake/Coral/Target Direction", intakeCoralObjective.getTargetDirection());
        Logger.recordOutput("Objective Tracker/Intake/Coral/Target Pose", intakeCoralObjective.getTargetPose().getOurs());
        Logger.recordOutput("Objective Tracker/Intake/Coral/Target Mechs", intakeCoralObjective.getTargetState().getMechTransforms());
        Logger.recordOutput("Objective Tracker/Intake/Algae", intakeAlgaeObjective.isPresent() ? intakeAlgaeObjective.get().getTargetAlgae().rack.id : -1);
        Logger.recordOutput("Objective Tracker/Intake/Algae/Target Direction", LoggerUtil.toArray(intakeAlgaeObjective.map(IntakeAlgaeObjective::getTargetDirection), Direction[]::new));
        Logger.recordOutput("Objective Tracker/Intake/Algae/Target Pose", LoggerUtil.toArray(intakeAlgaeObjective.map(IntakeAlgaeObjective::getTargetPose).map(AllianceFlipped::getOurs), Pose2d[]::new));
        Logger.recordOutput("Objective Tracker/Intake/Algae/Target Mechs", intakeAlgaeObjective.isPresent() ? intakeAlgaeObjective.get().getTargetState().getMechTransforms() : new Transform3d[0]);
        Logger.recordOutput("Objective Tracker/Score/Coral", scoreCoralObjective.getTargetBranch().isPresent() ? scoreCoralObjective.getTargetBranch().get().id : -1);
        Logger.recordOutput("Objective Tracker/Score/Coral/Target Direction", scoreCoralObjective.getTargetDirection());
        Logger.recordOutput("Objective Tracker/Score/Coral/Target Pose", scoreCoralObjective.getTargetPose().getOurs());
        Logger.recordOutput("Objective Tracker/Score/Coral/Target Mechs", scoreCoralObjective.getTargetState().getMechTransforms());
        Logger.recordOutput("Objective Tracker/Score/Algae/Target Direction", scoreAlgaeObjective.getTargetDirection());
        Logger.recordOutput("Objective Tracker/Score/Algae/Target Pose", scoreAlgaeObjective.getTargetPose().getOurs());
        Logger.recordOutput("Objective Tracker/Score/Algae/Target Mechs", scoreAlgaeObjective.getTargetState().getMechTransforms());
        Logger.recordOutput("Objective Tracker/Climb/Target Direction", climbObjective.getTargetDirection());
        Logger.recordOutput("Objective Tracker/Climb/Target Pose", climbObjective.getTargetPose().getOurs());
        Logger.recordOutput("Objective Tracker/Climb/Target Mechs", climbObjective.getTargetState().getMechTransforms());

        if (typeOverride.isEmpty()) {
            if (hasCoral && hasAlgae) {
                var distToCoral = scoreCoralObjective.getTargetPose().getOurs().getTranslation().getDistance(currentPose.getTranslation());
                var distToAlgae = scoreAlgaeObjective.getTargetPose().getOurs().getTranslation().getDistance(currentPose.getTranslation());
                if (distToCoral < distToAlgae) {
                    target = Optional.of(scoreCoralObjective);
                } else {
                    target = Optional.of(scoreAlgaeObjective);
                }
            } else if (hasCoral) {
                target = Optional.of(scoreCoralObjective);
            } else if (hasAlgae) {
                target = Optional.of(scoreAlgaeObjective);
            } else {
                target = Optional.of(intakeCoralObjective);
            }
        } else {
            target = switch (typeOverride.get()) {
                case IntakeCoral -> target = Optional.of(intakeCoralObjective);
                case IntakeAlgae -> target = intakeAlgaeObjective.map((objective) -> objective);
                case ScoreCoral -> target = Optional.of(scoreCoralObjective);
                case ScoreAlgae -> target = Optional.of(scoreAlgaeObjective);
                case Climb -> target = Optional.of(climbObjective);
            };
        }
    }

    public IntakeCoralObjective getIntakeCoralObjective() {
        return intakeCoralObjective;
    }
    public Optional<IntakeAlgaeObjective> getIntakeAlgaeObjective() {
        return intakeAlgaeObjective;
    }
    public ScoreCoralObjective getScoreCoralObjective() {
        return scoreCoralObjective;
    }
    public ScoreAlgaeObjective getScoreAlgaeObjective() {
        return scoreAlgaeObjective;
    }
    public ClimbObjective getClimbObjective() {
        return climbObjective;
    }
    public Optional<Objective> getCurrentObjective() {
        return target;
    }

    public void setTypeOverride(Optional<ObjectiveType> typeOverride) {
        this.typeOverride = typeOverride;
    }
    public Command setTypeOverrideCommand(ObjectiveType typeOverride) {
        return Commands.startEnd(() -> this.setTypeOverride(Optional.of(typeOverride)), () -> this.setTypeOverride(Optional.empty()));
    }

    // public void addTargetLock() {
    //     this.targetLocks += 1;
    // }
    // public void removeTargetLock() {
    //     this.targetLocks -= 1;
    // }
    // public Command addTargetLockCommand() {
    //     return Commands.startEnd(this::addTargetLock, this::removeTargetLock);
    // }

    public void addLevelLock(Optional<BranchLevel> level) {
        this.levelLock = Optional.of(level);
    }
    public void removeLevelLock() {
        this.levelLock = Optional.empty();
    }
    public void addPipeLock(PipeObject pipe) {
        this.pipeLock = Optional.of(pipe);
    }
    public void removePipeLock() {
        this.pipeLock = Optional.empty();
    }

    public static class BranchOrLevel1Object {
        private final BranchObject branch;
        private final RackObject level1Rack;

        private BranchOrLevel1Object(BranchObject branch, RackObject level1Rack) {
            this.branch = branch;
            this.level1Rack = level1Rack;
        }

        public static BranchOrLevel1Object fromBranch(BranchObject branch) {
            return new BranchOrLevel1Object(branch, null);
        }
        public static BranchOrLevel1Object fromLevel1(RackObject rack) {
            return new BranchOrLevel1Object(null, rack);
        }

        public BranchObject getBranch() {
            return branch;
        }
        public RackObject getLevel1Rack() {
            return level1Rack;
        }

        public boolean isBranch() {
            return branch != null;
        }
        public boolean isLevel1() {
            return level1Rack != null;
        }

        public Optional<BranchLevel> getBranchLevel() {
            if (isBranch()) {
                return Optional.of(getBranch().level);
            } else {
                return Optional.empty();
            }
        }

        public RobotFlippedRobotPose getPose() {
            if (isBranch()) {
                return getBranch().pipe.robotPose;
            } else {
                return getLevel1Rack().centerRobotPose;
            }
        }
    }
}