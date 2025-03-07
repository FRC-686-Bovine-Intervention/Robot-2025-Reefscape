package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.HashMap;
import java.util.Map;
import java.util.function.IntFunction;
import java.util.stream.IntStream;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.auto.AutoRoutine.AutoQuestion.Settings;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.CoralStation;
import frc.robot.constants.FieldConstants.Reef.Level;
import frc.robot.constants.FieldConstants.Reef.Pipe;
import frc.robot.constants.FieldConstants.Reef.Rack;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Direction;
import frc.util.flipping.AllianceFlipUtil;
import frc.util.flipping.AllianceFlipped;
import frc.util.misc.GeomUtil;

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

    public static Command followPathFlipped(PathPlannerPath path, Drive drive) {
        return new FollowPathCommand(path, drive::getPose, drive::getRobotMeasuredSpeeds, drive::drivePPVelocity, Drive.autoConfig(), DriveConstants.robotConfig, AllianceFlipUtil::shouldFlip, drive.translationSubsystem, drive.rotationalSubsystem)
            .deadlineFor(Commands.startEnd(
                () -> Logger.recordOutput("Autonomous/Goal Pose", AllianceFlipUtil.apply(new Pose2d(getLastPoint(path), path.getGoalEndState().rotation()))),
                () -> Logger.recordOutput("Autonomous/Goal Pose", (Pose2d)null)
            ))
        ;
    }
    public static Command followPathFlipped(PathPlannerPath path, Drive.Translational drive) {
        return new FollowPathCommand(path, drive.drive::getPose, drive.drive::getRobotMeasuredSpeeds, drive.drive::drivePPVelocity, Drive.autoConfig(), DriveConstants.robotConfig, AllianceFlipUtil::shouldFlip, drive)
            .deadlineFor(Commands.startEnd(
                () -> Logger.recordOutput("Autonomous/Goal Pose", AllianceFlipUtil.apply(new Pose2d(getLastPoint(path), path.getGoalEndState().rotation()))),
                () -> Logger.recordOutput("Autonomous/Goal Pose", (Pose2d)null)
            ))
        ;
    }

    public static Command scoreOnReef(PathPlannerPath pathToReef, Level branchLevel, Direction direction, Drive drive, Superstructure superstructure, Intake intake) {
        var targetState = branchLevel.superstructureStates.get(direction);
        var end = getLastPoint(pathToReef);
        return 
            Commands.deadline(
                Commands.sequence(
                    Commands.waitUntil(() -> superstructure.getCurrentState().isNear(targetState, Degrees.of(1), Inches.of(1), Degrees.of(3))),
                    Commands.waitSeconds(1),
                    intake.eject().asProxy().withTimeout(0.75)//.onlyWhile(intake.hasCoral)
                ),
                Commands.sequence(
                    Commands.waitUntil(() -> GeomUtil.isNear(end, drive.getPose().getTranslation(), Inches.of(48))),
                    superstructure.goToSetpointSequenced(targetState).asProxy()
                ),
                followPathFlipped(pathToReef, drive).asProxy()
            )
        ;
    }

    public static Command scoreInNet(PathPlannerPath pathToBarge, Direction direction, Drive drive, Superstructure superstructure, Intake intake) {
        return Commands.none();
    }

    public static Command scoreInProcessor(PathPlannerPath pathToProcessor, Drive drive, Superstructure superstructure, Intake intake) {
        return Commands.none();
    }

    public static Command pickupCoralFromStation(PathPlannerPath pathToStation, Direction direction, Drive drive, Superstructure superstructure, Intake intake) {
        return 
            Commands.deadline(
                intake.intake().asProxy().until(intake.hasCoral),
                followPathFlipped(pathToStation, drive).asProxy(),
                superstructure.goToSetpointSequenced(CoralStation.intakePosition.get(direction)).asProxy()
            )
        ;
    }

    public static Command pickupAlgaeFromReef(PathPlannerPath pathToReef, Direction direction, Drive drive, Superstructure superstructure, Intake intake) {
        return Commands.none();
    }

    public static final Map.Entry<String, Pipe>[] pipeOptions =
        IntStream.range(0, FieldConstants.Reef.pipes.length)
            .mapToObj(i -> Settings.option(String.valueOf(FieldConstants.Reef.pipes[i].getLetter()), FieldConstants.Reef.pipes[i]))
            .toArray((IntFunction<Map.Entry<String, Pipe>[]>) Map.Entry[]::new)
    ;
    
    public static final Map.Entry<String, Rack>[] rackOptions = 
        IntStream.range(0, FieldConstants.Reef.Rack.values().length)
        .mapToObj(i -> Settings.option("Rack " + i, FieldConstants.Reef.Rack.values()[i]))
        .toArray((IntFunction<Map.Entry<String, Rack>[]>) Map.Entry[]::new);

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

        public static String getName(PathPlannerPath path) {
            return loadedPaths.entrySet().stream().filter((e) -> e.getValue() == path).map((e) -> e.getKey()).findAny().orElse("Unknown Path");
        }
    }
}