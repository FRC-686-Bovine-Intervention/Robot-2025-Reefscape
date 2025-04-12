package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.IntFunction;
import java.util.stream.IntStream;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.auto.AutoRoutine.AutoQuestion.Settings;
import frc.robot.constants.FieldConstants.Barge;
import frc.robot.constants.FieldConstants.CoralStation;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Reef.BranchConcept;
import frc.robot.constants.FieldConstants.Reef.PipeConcept;
import frc.robot.constants.FieldConstants.Reef.StagedAlgaeConcept;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.superstructure.Superstructure.Direction;
import frc.util.flipping.AllianceFlipUtil;
import frc.util.flipping.AllianceFlipped;
import frc.util.geometry.GeomUtil;

public class AutoCommons {
    public static Translation2d getFirstPoint(PathPlannerPath path) {
        return path.getPoint(0).position;
    }

    public static Translation2d getLastPoint(PathPlannerPath path) {
        return path.getPoint(path.numPoints() - 1).position;
    }

    public static Command setOdometryFlipped(AllianceFlipped<Pose2d> pose, Drive drive) {
        return Commands.runOnce(() -> RobotState.getInstance().setPose(drive.getGyroRotation(), drive.getModulePositions(), pose.getOurs()));
    }

    // public static Command followPathFlipped(PathPlannerPath path, Drive drive) {
    //     return new FollowPathCommand(path, drive::getPose, drive::getRobotMeasuredSpeeds, drive::drivePPVelocity, Drive.autoConfig(), DriveConstants.robotConfig, AllianceFlipUtil::shouldFlip, drive.translationSubsystem, drive.rotationalSubsystem)
    //         .deadlineFor(Commands.startEnd(
    //             () -> Logger.recordOutput("Autonomous/Goal Pose", AllianceFlipUtil.apply(new Pose2d(getLastPoint(path), path.getGoalEndState().rotation()))),
    //             () -> Logger.recordOutput("Autonomous/Goal Pose", (Pose2d)null)
    //         ))
    //     ;
    // }
    // public static Command followPathFlipped(PathPlannerPath path, Drive.Translational drive) {
    //     return new FollowPathCommand(path, drive.drive::getPose, drive.drive::getRobotMeasuredSpeeds, drive.drive::drivePPVelocity, Drive.autoConfig(), DriveConstants.robotConfig, AllianceFlipUtil::shouldFlip, drive)
    //         .deadlineFor(Commands.startEnd(
    //             () -> Logger.recordOutput("Autonomous/Goal Pose", AllianceFlipUtil.apply(new Pose2d(getLastPoint(path), path.getGoalEndState().rotation()))),
    //             () -> Logger.recordOutput("Autonomous/Goal Pose", (Pose2d)null)
    //         ))
    //     ;
    // }

    public static Command scoreOnReef(PathPlannerPath pathToReef, BranchConcept branch, Direction direction, Drive drive, Superstructure superstructure, Intake intake) {
        var targetState = branch.level.scoringSuperstructureStates.get(direction);
        var endTranslation = AllianceFlipUtil.apply(getLastPoint(pathToReef));
        var endRotation = AllianceFlipUtil.apply(pathToReef.getGoalEndState().rotation());
        var end = new Pose2d(endTranslation, endRotation);
        return 
            Commands.deadline(
                Commands.sequence(
                    Commands.waitUntil(() -> superstructure.getCurrentState().isNear(targetState, Degrees.of(2), Inches.of(1), Degrees.of(5))),
                    Commands.waitUntil(() -> GeomUtil.isNear(end, drive.getPose(), Inches.of(5), Degrees.of(5))),
                    Commands.waitSeconds(0.25),
                    intake.eject().asProxy().onlyWhile(intake.hasCoral)
                ),
                Commands.sequence(
                    Commands.waitUntil(() -> GeomUtil.isNear(endTranslation, drive.getPose().getTranslation(), Feet.of(6))),
                    superstructure.goToSetpointSequenced(targetState).withName("Extend to " + branch.getName()).asProxy()
                ),
                Commands.sequence(
                    drive.followBluePath(pathToReef).withName("Follow Path to " + branch.getName()).asProxy(),
                    drive.simplePIDTo(() -> end).withName("PID to " + branch.getName()).asProxy()
                )
            )
        ;
    }

    public static Command scoreInNet(PathPlannerPath pathToBarge, Direction direction, Drive drive, Superstructure superstructure, Intake intake) {
        var targetState = Barge.superstructureState.get(direction);
        var endTranslation = AllianceFlipUtil.apply(getLastPoint(pathToBarge));
        var endRotation = AllianceFlipUtil.apply(pathToBarge.getGoalEndState().rotation());
        var end = new Pose2d(endTranslation, endRotation);
        return
            Commands.deadline(
                Commands.sequence(
                    Commands.waitUntil(() -> superstructure.getCurrentState().isNear(targetState, Degrees.of(2), Inches.of(1), Degrees.of(5))),
                    Commands.waitUntil(() -> GeomUtil.isNear(end, drive.getPose(), Inches.of(5), Degrees.of(5))),
                    Commands.waitSeconds(0.5),
                    intake.eject().asProxy().onlyWhile(intake.hasAlgae.debounce(0.75, DebounceType.kFalling))
                ),
                Commands.sequence(
                    Commands.waitUntil(() -> GeomUtil.isNear(endTranslation, drive.getPose().getTranslation(), Feet.of(6))),
                    superstructure.goToSetpointSequenced(targetState).withName("Extend to Net").asProxy()
                ),
                Commands.sequence(
                    drive.followBluePath(pathToBarge).withName("Follow Path to Net").asProxy(),
                    drive.simplePIDTo(() -> end).withName("PID to Net").asProxy()
                )
            )
        ;
    }
    public static Command scoreInNet(PathPlannerPath pathToExtend, PathPlannerPath pathToNet, Direction direction, Drive drive, Superstructure superstructure, Intake intake) {
        var targetState = Barge.superstructureState.get(direction);
        var extendPose = new Pose2d(AllianceFlipUtil.apply(getLastPoint(pathToExtend)), AllianceFlipUtil.apply(pathToExtend.getGoalEndState().rotation()));
        var netPose = new Pose2d(AllianceFlipUtil.apply(getLastPoint(pathToNet)), AllianceFlipUtil.apply(pathToNet.getGoalEndState().rotation()));
        return
            Commands.deadline(
                Commands.sequence(
                    Commands.waitUntil(() -> 
                        GeomUtil.isNear(netPose, drive.getPose(), Inches.of(5), Degrees.of(5))
                        && superstructure.getCurrentState().isNear(targetState, Degrees.of(2), Inches.of(6), Degrees.of(5))
                    ),
                    intake.eject().asProxy().onlyWhile(intake.hasAlgae.debounce(0.75, DebounceType.kFalling))
                ),
                Commands.sequence(
                    Commands.sequence(
                        drive.followBluePath(pathToExtend).asProxy(),
                        drive.simplePIDTo(() -> extendPose).asProxy()
                    ).until(() -> superstructure.getCurrentState().isNear(targetState, Degrees.of(2), Inches.of(6), Degrees.of(45))),
                    Commands.sequence(
                        drive.followBluePath(pathToNet).asProxy(),
                        drive.simplePIDTo(() -> netPose).asProxy()
                    )
                ),
                Commands.sequence(
                    superstructure.goToSetpointSequenced(SuperstructureConstants.netPrepareState).until(() -> GeomUtil.isNear(extendPose, drive.getPose(), Inches.of(5), Degrees.of(10))).asProxy(),
                    superstructure.goToSetpointSequenced(targetState).asProxy()
                )
            )
        ;
    }

    public static Command scoreInProcessor(PathPlannerPath pathToProcessor, Drive drive, Superstructure superstructure, Intake intake) {
        return Commands.none();
    }

    public static Command pickupCoralFromStation(PathPlannerPath pathToStation, Direction direction, Drive drive, Superstructure superstructure, Intake intake) {
        var endTranslation = AllianceFlipUtil.apply(getLastPoint(pathToStation));
        var endRotation = AllianceFlipUtil.apply(pathToStation.getGoalEndState().rotation());
        var end = new Pose2d(endTranslation, endRotation);
        if (RobotBase.isReal()) {
            return 
                Commands.deadline(
                    intake.intakeCoral().asProxy().until(intake.hasCoral),
                    Commands.sequence(
                        drive.followBluePath(pathToStation).withName("Follow Path to Coral Station").asProxy(),
                        drive.simplePIDTo(() -> end).withName("PID to Coral Station").asProxy()
                    ),
                    superstructure.goToSetpointSequenced(CoralStation.intakePosition.get(direction)).withName("Extend to Coral Station").asProxy()
                )
            ;
        } else {
            return 
                Commands.deadline(
                    intake.intakeCoral().asProxy().withTimeout(4).until(intake.hasCoral),
                    Commands.sequence(
                        drive.followBluePath(pathToStation).withName("Follow Path to Coral Station").asProxy(),
                        drive.simplePIDTo(() -> end).withName("PID to Coral Station").asProxy()
                    ),
                    superstructure.goToSetpointSequenced(CoralStation.intakePosition.get(direction)).withName("Extend to Coral Station").asProxy()
                )
            ;
        }
    }

    public static Command pickupAlgaeFromReef(PathPlannerPath pathToReef, StagedAlgaeConcept stagedAlgae, Direction direction, Drive drive, Superstructure superstructure, Intake intake) {
        var endTranslation = AllianceFlipUtil.apply(getLastPoint(pathToReef));
        var endRotation = AllianceFlipUtil.apply(pathToReef.getGoalEndState().rotation());
        var end = new Pose2d(endTranslation, endRotation);
        if (RobotBase.isReal()) {
            return 
                Commands.deadline(
                    Commands.sequence(
                        intake.intakeAlgae().asProxy().until(intake.hasAlgae),
                        Commands.waitSeconds(0.5)
                    ),
                    Commands.sequence(
                        drive.followBluePath(pathToReef).withName("Follow Path to Algae " + stagedAlgae.rack.id).asProxy(),
                        drive.simplePIDTo(() -> end).withName("PID to Algae " + stagedAlgae.rack.id).asProxy()
                    ),
                    superstructure.goToSetpointSequenced(stagedAlgae.level.intakeSuperstructureStates.get(direction)).withName("Extend to " + stagedAlgae.level.name() + " Algae").asProxy()
                )
            ;
        } else {
            return 
                Commands.deadline(
                    intake.intakeAlgae().asProxy().withTimeout(2).until(intake.hasAlgae),
                    Commands.sequence(
                        drive.followBluePath(pathToReef).withName("Follow Path to Algae " + stagedAlgae.rack.id).asProxy(),
                        drive.simplePIDTo(() -> end).withName("PID to Algae " + stagedAlgae.rack.id).asProxy()
                    ),
                    superstructure.goToSetpointSequenced(stagedAlgae.level.intakeSuperstructureStates.get(direction)).withName("Extend to " + stagedAlgae.level.name() + " Algae").asProxy()
                )
            ;
        }
    }
    public static Command pickupAlgaeFromReef(StagedAlgaeConcept stagedAlgae, Direction direction, Drive drive, Superstructure superstructure, Intake intake) {
        var backupTransform = new Transform2d(new Translation2d(Inches.of(12).unaryMinus(), Inches.zero()), Rotation2d.kZero);

        var intakePose = stagedAlgae.rack.getOurs().centerRobotPose.get(direction);
        var backupPose = intakePose.transformBy(backupTransform);

        var targetState = stagedAlgae.level.intakeSuperstructureStates.get(direction);
        if (RobotBase.isReal()) {
            return 
                Commands.deadline(
                    Commands.sequence(
                        intake.intakeAlgae().asProxy().until(intake.hasAlgae),
                        Commands.waitSeconds(0.5)
                    ),
                    Commands.sequence(
                        drive.simplePIDTo(() -> backupPose).withName("Backup").asProxy().until(() -> superstructure.getCurrentState().isNear(targetState, Degrees.of(5), Inches.of(5), Degrees.of(5))),
                        drive.simplePIDTo(() -> intakePose).withName("Intake").asProxy()
                        // drive.followBluePath(pathToReef).withName("Follow Path to Algae " + stagedAlgae.rack.id).asProxy(),
                        // drive.simplePIDTo(() -> end).withName("PID to Algae " + stagedAlgae.rack.id).asProxy()
                    ),
                    superstructure.goToSetpointSequenced(stagedAlgae.level.intakeSuperstructureStates.get(direction)).withName("Extend to " + stagedAlgae.level.name() + " Algae").asProxy()
                )
            ;
        } else {
            return 
                Commands.none()
                // Commands.deadline(
                //     intake.intakeAlgae().asProxy().withTimeout(2).until(intake.hasAlgae),
                //     Commands.sequence(
                //         drive.followBluePath(pathToReef).withName("Follow Path to Algae " + stagedAlgae.rack.id).asProxy(),
                //         drive.simplePIDTo(() -> end).withName("PID to Algae " + stagedAlgae.rack.id).asProxy()
                //     ),
                //     superstructure.goToSetpointSequenced(stagedAlgae.level.intakeSuperstructureStates.get(direction)).withName("Extend to " + stagedAlgae.level.name() + " Algae").asProxy()
                // )
            ;
        }
    }

    public static final Map.Entry<String, PipeConcept>[] pipeOptions = Arrays.stream(Reef.pipes)
        .map((pipe) -> Settings.option(pipe.getLetter(), pipe))
        .toArray((IntFunction<Map.Entry<String, PipeConcept>[]>) Map.Entry[]::new)
    ;
    public static final Map.Entry<String, Optional<PipeConcept>>[] pipeOptionalOptions =
        IntStream.range(0, 13)
        .mapToObj((i) -> {
            if (i < Reef.pipes.length) {
                return Settings.option(Reef.pipes[i].getLetter(), Optional.of(Reef.pipes[i]));
            } else {
                return Settings.option("None", Optional.empty());
            }
        })
        .toArray((IntFunction<Map.Entry<String, Optional<PipeConcept>>[]>) Map.Entry[]::new)
    ;
    
    public static final Map.Entry<String, StagedAlgaeConcept>[] algaeOptions = 
        IntStream.range(0, 6)
        .mapToObj((i) -> Settings.option("Rack " + i, Reef.stagedAlgae[i]))
        .toArray((IntFunction<Map.Entry<String, StagedAlgaeConcept>[]>) Map.Entry[]::new)
    ;
    public static final Map.Entry<String, Optional<StagedAlgaeConcept>>[] algaeOptionalOptions =
        IntStream.range(0, 7)
        .mapToObj((i) -> {
            if (i < Reef.stagedAlgae.length) {
                return Settings.option("Rack " + i, Optional.of(Reef.stagedAlgae[i]));
            } else {
                return Settings.option("None", Optional.empty());
            }
        })
        .toArray((IntFunction<Map.Entry<String, Optional<StagedAlgaeConcept>>[]>) Map.Entry[]::new)
    ;

    public static enum BargePosition {
        LEFT,
        CENTER,
        RIGHT
    }
    public static enum CoralStationPosition {
        CLOSE,
        MID,
        FAR
    }

    public static String getBargePositionAsString(BargePosition _bargePosition){
        return switch(_bargePosition){
            case LEFT -> "BargeLeft";
            case CENTER -> "BargeCenter";
            case RIGHT -> "BargeRight";
            default -> null;
        };
    }

    public static String getCoralStationPositionAsString(CoralStationPosition _stationPosition){
        return switch (_stationPosition) {
            case CLOSE -> "Close";
            case MID -> "Mid";
            case FAR -> "Far";
            default -> null;
        };
    }

    public static String getStartingPositionAsString(AllianceFlipped<Pose2d> startingPosition){
        if(startingPosition == AutoConstants.startDeadCenter){
            return "StartDeadCenter";
        } else if(startingPosition == AutoConstants.startLeftLeftCage){
            return "StartLeftLeftCage";
        } else if(startingPosition == AutoConstants.startLeftMiddleCage){
            return "StartLeftMiddleCage";
        } else if(startingPosition == AutoConstants.startLeftRightCage){
            return "StartLeftRightCage";
        } else if(startingPosition == AutoConstants.startRightLeftCage){
            return "StartRightLeftCage";
        } else if(startingPosition == AutoConstants.startRightMiddleCage){
            return "StartRightMiddleCage";
        } else if(startingPosition == AutoConstants.startRightRightCage){
            return "StartRightRightCage";
        } else{
            return null;
        }
    }

    public static class AutoPaths {
        private static final Map<String, PathPlannerPath> loadedPaths = new HashMap<>();
        private static boolean preloading;
        public static void preload() {
            preloading = true;
            // load paths
            // loadChoreoTrajectory("Inner To Stacking");
            preloading = false;
            System.out.println("[Init AutoPaths] Loaded paths");
            PathPlannerLogging.setLogActivePathCallback((path) -> Logger.recordOutput("Autonomous/Path", path.toArray(Pose2d[]::new)));
            PathPlannerLogging.setLogTargetPoseCallback((target) -> Logger.recordOutput("Autonomous/Target Pose", target));
        }

        @SuppressWarnings("resource")
        public static PathPlannerPath loadPath(String name) {
            if(loadedPaths.containsKey(name)) {
                return loadedPaths.get(name);
            } else {
                // if(!preloading) new Alert("[AutoPaths] Loading \"" + name + "\" which wasn't preloaded. Please add path to AutoPaths.preload()", AlertType.kWarning).set(true);
                try {
                    var path = PathPlannerPath.fromPathFile(name);
                    // loadedPaths.put(name, path);
                    return path;
                } catch (Exception e) {
                    return null;
                }
            }
        }

        @SuppressWarnings("resource")
        public static PathPlannerPath loadChoreoTrajectory(String name) {
            if(loadedPaths.containsKey(name)) {
                return loadedPaths.get(name);
            } else {
                // if(!preloading) new Alert("[AutoPaths] Loading \"" + name + "\" which wasn't preloaded. Please add path to AutoPaths.preload()", AlertType.kWarning).set(true);
                try {
                    var path = PathPlannerPath.fromChoreoTrajectory(name);
                    // loadedPaths.put(name, path);
                    return path;
                } catch (Exception e) {
                    return null;
                }
            }
        }
        @SuppressWarnings("resource")
        public static PathPlannerPath loadChoreoTrajectory(String name, int splitIndex) {
            if(loadedPaths.containsKey(name)) {
                return loadedPaths.get(name);
            } else {
                // if(!preloading) new Alert("[AutoPaths] Loading \"" + name + "\" which wasn't preloaded. Please add path to AutoPaths.preload()", AlertType.kWarning).set(true);
                try {
                    var path = PathPlannerPath.fromChoreoTrajectory(name, splitIndex);
                    // loadedPaths.put(name, path);
                    return path;
                } catch (Exception e) {
                    return null;
                }
            }
        }

        public static String getName(PathPlannerPath path) {
            return loadedPaths.entrySet().stream().filter((e) -> e.getValue() == path).map((e) -> e.getKey()).findAny().orElse("Unknown Path");
        }
    }
}