package frc.robot.auto;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.function.IntFunction;
import java.util.stream.IntStream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoRoutine.AutoQuestion.Settings;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.CoralStation;
import frc.robot.constants.FieldConstants.Reef.Level;
import frc.robot.constants.FieldConstants.Reef.Pipe;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.util.flipping.AllianceFlipped;
import frc.util.misc.MathExtraUtil;

public class ScoreCoral extends AutoRoutine {
    // scoring preload (reef pipes)
    // starting position (closest (center pillar), far?)
    // scoring coral 1 (1/2 reef pipes - 1)
    // scoring coral 2 (1/2 reef pipes - 2)
    // which part of the coral station (close, mid, far)
    
    private static final Map.Entry<String, Pipe>[] pipeOptions =
        IntStream.range(0, FieldConstants.Reef.pipes.length)
            .mapToObj(i -> Settings.option("Pipe " + FieldConstants.Reef.pipes[i].getLetter(), FieldConstants.Reef.pipes[i]))
            .toArray((IntFunction<Map.Entry<String, Pipe>[]>) Map.Entry[]::new);

    private static boolean isRightCoralStation(Pipe pipe){
        return MathExtraUtil.isWithin(pipe.getIndex(), 1, 6);
    }

    private static final AutoQuestion<Pipe> scorePreloadPipe = new AutoQuestion<Pipe>("Score Preload Pipe") {
        @Override
        protected Settings<Pipe> generateSettings() {
            var options = Arrays
                .stream(pipeOptions, 2, pipeOptions.length)
                .sorted((e1, e2) -> e1.getKey().compareTo(e2.getKey()))
                .toArray((IntFunction<Map.Entry<String, Pipe>[]>) Map.Entry[]::new);
            return Settings.from(
                options[0],
                options
            );
        }

    };

    private static final AutoQuestion<Pipe> scoreCoral1 = new AutoQuestion<Pipe>("Score Second Pipe") {
        @Override
        protected Settings<Pipe> generateSettings() {
            var options = (
                isRightCoralStation(scorePreloadPipe.getResponse())
                ? IntStream.range(1, 7)
                : IntStream.range(7, 13)
            )
                .mapToObj(i -> pipeOptions[i % pipeOptions.length])
                .filter(entry -> !entry.getValue().equals(scorePreloadPipe.getResponse()))
                .sorted((e1, e2) -> e1.getKey().compareTo(e2.getKey()))
                .toArray((IntFunction<Map.Entry<String, Pipe>[]>) Map.Entry[]::new);
            return Settings.from(options[0], options);
        }
    };

    private static final AutoQuestion<Pipe> scoreCoral2 = new AutoQuestion<Pipe>("Score Third Pipe") {
        @Override
        protected Settings<Pipe> generateSettings() {
            var options = (
                isRightCoralStation(scorePreloadPipe.getResponse())
                ? IntStream.range(1, 7)
                : IntStream.range(7, 13)
            )
                .mapToObj(i -> pipeOptions[i % pipeOptions.length])
                .filter(entry -> 
                    !entry.getValue().equals(scorePreloadPipe.getResponse()) &&
                    !entry.getValue().equals(scoreCoral1.getResponse())
                )
                .sorted((e1, e2) -> e1.getKey().compareTo(e2.getKey()))
                .toArray((IntFunction<Map.Entry<String, Pipe>[]>) Map.Entry[]::new);
            return Settings.from(options[0], options);
        }
    };

    private enum CoralStationPosition{
        CLOSE,
        MID,
        FAR
    }
    private static final AutoQuestion<CoralStationPosition> stationPosition = new AutoQuestion<CoralStationPosition>("Coral Station Position") {
        private static final Map.Entry<String, CoralStationPosition> stationFar = Settings.option("Far", CoralStationPosition.FAR);
        private static final Map.Entry<String, CoralStationPosition> stationMid = Settings.option("Mid", CoralStationPosition.MID);
        private static final Map.Entry<String, CoralStationPosition> stationClose = Settings.option("Close", CoralStationPosition.CLOSE);
        
        @Override
        protected Settings<CoralStationPosition> generateSettings() {
            return Settings.from(stationClose, stationClose, stationMid, stationFar);
        }
    };

    private static final AutoQuestion<AllianceFlipped<Pose2d>> startPosition = new AutoQuestion<AllianceFlipped<Pose2d>>("Starting Position") {
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startRemoteLeft = Settings.option("Remote (Left)", AutoConstants.startRemoteLeft);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startRemoteRight = Settings.option("Remote (Right)", AutoConstants.startRemoteRight);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startFarLeft = Settings.option("Close (Far Left)", AutoConstants.startFarLeft);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startFarRight = Settings.option("Close (Far Right)", AutoConstants.startFarRight);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startLeftCage = Settings.option("Close (Left Cage)", AutoConstants.startLeftCage);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startRightCage = Settings.option("Close(Right Cage)", AutoConstants.startRightCage);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startLeftCenter = Settings.option("Close (Left Center)", AutoConstants.startLeftCenter);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startRightCenter = Settings.option("Close (Right Center)", AutoConstants.startRightCenter);
        
        @Override
        protected Settings<AllianceFlipped<Pose2d>> generateSettings() {
            switch(scorePreloadPipe.getResponse().getIndex()){
                case 0:
                return Settings.from(startFarLeft, startFarLeft, startRemoteLeft);
                case 1:
                return Settings.from(startFarRight, startFarRight, startRemoteRight);
                case 2:
                return Settings.from(startFarRight, startFarRight, startRemoteRight);
                case 3:
                return Settings.from(startFarRight, startFarRight, startRemoteRight);
                case 4:
                return Settings.from(startFarRight, startFarRight);
                case 5:
                return Settings.from(startRightCage, startRightCage);
                case 6:
                return Settings.from(startRightCenter, startRightCenter);
                case 7:
                return Settings.from(startLeftCenter, startLeftCenter);
                case 8:
                return Settings.from(startLeftCage, startLeftCage);
                case 9:
                return Settings.from(startFarLeft, startFarLeft);
                case 10:
                return Settings.from(startFarLeft, startFarLeft, startRemoteLeft);
                case 11:
                return Settings.from(startFarLeft, startFarLeft, startRemoteLeft);
                default:
                return null;
            }
        }
    };

    private final Drive drive;
    private final Superstructure superstructure;
    private final Intake intake;

    public ScoreCoral(RobotContainer robot) {
        super("ScoreCoral", List.of(scorePreloadPipe, startPosition, scoreCoral1, scoreCoral2, stationPosition));
        this.drive = robot.drive;
        this.superstructure = robot.superstructure;
        this.intake = robot.intake;
    }
    
    @Override
    public Command generateCommand() {
        var scorePreloadPipe = ScoreCoral.scorePreloadPipe.getResponse();
        var startPosition = ScoreCoral.startPosition.getResponse();
        var scoreCoral1 = ScoreCoral.scoreCoral1.getResponse();
        var scoreCoral2 = ScoreCoral.scoreCoral2.getResponse();
        var stationPosition = ScoreCoral.stationPosition.getResponse();
        var commands = new ArrayList<Command>();

        String startToScorePath;
        if (
            startPosition.equals(AutoConstants.startRemoteLeft) ||
            startPosition.equals(AutoConstants.startRemoteRight)
        ) {
            startToScorePath = "Remote Start To " + getBranchLetterFromIndex(scorePreloadPipe.getIndex());
        } else {
            startToScorePath = "Start To " + getBranchLetterFromIndex(scorePreloadPipe.getIndex());
        }
        var startToScorePreload = AutoPaths.loadChoreoTrajectory(startToScorePath);
        commands.add(Commands.sequence(
            Commands.parallel(
                superstructure.goToSetpointSequenced(Level.Level4.superstructureStates.getForward())
            ),
            intake.eject().until(intake.hasCoral)
        ));
        var preloadToStation = AutoPaths.loadChoreoTrajectory(
            getBranchLetterFromIndex(scorePreloadPipe.getIndex()) +
            " To Station " +
            getStationPositionAsString(stationPosition)
        );
        commands.add(Commands.parallel(
            drive.followBluePath(preloadToStation),
            superstructure.goToSetpointSequenced(CoralStation.intakePosition.getForward()),
            intake.intake().until(intake.hasCoral)
        ));
        var stationToScore1 = AutoPaths.loadChoreoTrajectory(
            "Station "
            + getStationPositionAsString(stationPosition)
            + " To "
            + getBranchLetterFromIndex(scoreCoral1.getIndex())
        );
        commands.add(Commands.sequence(
            Commands.parallel(
                drive.followBluePath(stationToScore1),
                superstructure.goToSetpointSequenced(Level.Level4.superstructureStates.getForward())
            ),
            intake.eject().until(intake.hasCoral.negate())
        ));
        var coral1ToStation = AutoPaths.loadChoreoTrajectory(
            getBranchLetterFromIndex(scoreCoral1.getIndex())+
            " To Station "+
            getStationPositionAsString(stationPosition)
        );
        commands.add(
            Commands.parallel(
                drive.followBluePath(coral1ToStation),
                superstructure.goToSetpointSequenced(CoralStation.intakePosition.getForward()),
                intake.intake().until(intake.hasCoral)
            )
        );
        var stationToScore2 = AutoPaths.loadChoreoTrajectory(
            "Station "
            + getStationPositionAsString(stationPosition)
            + " To "
            + getBranchLetterFromIndex(scoreCoral2.getIndex())
        );
        commands.add(Commands.sequence(
            Commands.parallel(
                drive.followBluePath(stationToScore2),
                superstructure.goToSetpointSequenced(Level.Level4.superstructureStates.getForward()
            )),
            intake.eject().until(intake.hasCoral.negate())
        ));
        return AutoCommons
            .setOdometryFlipped(startPosition, drive)
            .andThen(commands.toArray(Command[]::new));
    }

    private static char getBranchLetterFromIndex(int index){
        return (char) (index + 'A');
    }
    private String getStationPositionAsString(CoralStationPosition _stationPosition){
        return switch (_stationPosition) {
            case CLOSE -> "Close";
            case MID -> "Mid";
            case FAR -> "Far";
            default -> null;
        };
    }
}
