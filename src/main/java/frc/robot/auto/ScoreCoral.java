package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.IntFunction;
import java.util.stream.IntStream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoRoutine.AutoQuestion.Settings;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef.Pipe;
import frc.robot.subsystems.drive.Drive;
import frc.util.flipping.Flipped;
import frc.util.misc.MathExtraUtil;

public class ScoreCoral extends AutoRoutine {
    // scoring preload (reef pipes)
    // starting position (closest (center pillar), far?)
    // scoring coral 1 (1/2 reef pipes - 1)
    // scoring coral 2 (1/2 reef pipes - 2)
    // which part of the coral station (close, mid, far)
    
    private static final Map.Entry<String, Pipe>[] pipeOptions =
        IntStream.range(0, FieldConstants.Reef.pipes.length)
            .mapToObj(i -> Settings.option("Pipe " + getBranchLetterFromIndex(i), FieldConstants.Reef.pipes[i]))
            .toArray((IntFunction<Map.Entry<String, Pipe>[]>) Map.Entry[]::new);

    private static boolean isRightCoralStation(Pipe pipe){
        return MathExtraUtil.isWithin(pipe.getIndex(), 1, 6);
    }

    private static final AutoQuestion<Pipe> scorePreloadPipe = new AutoQuestion<Pipe>("Score Preload Pipe") {
        @Override
        protected Settings<Pipe> generateSettings() {
            return Settings.from(
                pipeOptions[0],
                pipeOptions
            );
        }

    };

    private static final AutoQuestion<Pipe> scoreCoral1 = new AutoQuestion<Pipe>("Score Second Pipe") {
        @Override
        protected Settings<Pipe> generateSettings() {
            var options = (
                isRightCoralStation(scorePreloadPipe.getResponse())
                ? IntStream.range(1, 7)
                : IntStream.range(7, 12)
            )
                .mapToObj(i -> pipeOptions[i % pipeOptions.length])
                .filter(entry -> !entry.getValue().equals(scorePreloadPipe.getResponse()))
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
                : IntStream.range(7, 12)
            )
                .mapToObj(i -> pipeOptions[i % pipeOptions.length])
                .filter(entry -> 
                    !entry.getValue().equals(scorePreloadPipe.getResponse()) &&
                    !entry.getValue().equals(scoreCoral1.getResponse())
                )
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

    private static final AutoQuestion<Flipped<Pose2d>> startPosition = new AutoQuestion<Flipped<Pose2d>>("Starting Position") {
        private static final Map.Entry<String, Flipped<Pose2d>> startRemoteLeft = Settings.option("Remote (Left)", AutoConstants.startRemoteLeft);
        private static final Map.Entry<String, Flipped<Pose2d>> startRemoteRight = Settings.option("Remote (Right)", AutoConstants.startRemoteRight);
        private static final Map.Entry<String, Flipped<Pose2d>> startFarLeft = Settings.option("Close (Far Left)", AutoConstants.startFarLeft);
        private static final Map.Entry<String, Flipped<Pose2d>> startFarRight = Settings.option("Close (Far Right)", AutoConstants.startFarRight);
        private static final Map.Entry<String, Flipped<Pose2d>> startLeftCage = Settings.option("Close (Left Cage)", AutoConstants.startLeftCage);
        private static final Map.Entry<String, Flipped<Pose2d>> startRightCage = Settings.option("Close(Right Cage)", AutoConstants.startRightCage);
        private static final Map.Entry<String, Flipped<Pose2d>> startLeftCenter = Settings.option("Close (Left Center)", AutoConstants.startLeftCenter);
        private static final Map.Entry<String, Flipped<Pose2d>> startRightCenter = Settings.option("Close (Right Center)", AutoConstants.startRightCenter);
        
        @Override
        protected Settings<Flipped<Pose2d>> generateSettings() {
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
    public ScoreCoral(RobotContainer robot) {
        super("ScoreCoral", List.of(scorePreloadPipe, startPosition, scoreCoral1, scoreCoral2, stationPosition));
        this.drive = robot.drive;
    }
    
    @Override
    public Command generateCommand() {
        var _scorePreloadPipe = scorePreloadPipe.getResponse();
        var _startPosition = startPosition.getResponse();
        var _scoreCoral1 = scoreCoral1.getResponse();
        var _scoreCoral2 = scoreCoral2.getResponse();
        var _stationPosition = stationPosition.getResponse();
        var commands = new ArrayList<Command>(1);

        String startToScorePath;
        if (
            _startPosition.equals(AutoConstants.startRemoteLeft) ||
            _startPosition.equals(AutoConstants.startRemoteRight)
        ) {
            startToScorePath = "Remote Start To " + getBranchLetterFromIndex(_scorePreloadPipe.getIndex());
        } else {
            startToScorePath = "Start To " + getBranchLetterFromIndex(_scorePreloadPipe.getIndex());
        }
        var startToScorePreload = AutoPaths.loadChoreoTrajectory(startToScorePath);
        commands.add(drive.followBluePath(startToScorePreload));
        var preloadToStation = AutoPaths.loadChoreoTrajectory(
            getBranchLetterFromIndex(_scorePreloadPipe.getIndex()) +
            " To Station " +
            getStationPositionAsString(_stationPosition)
        );
        commands.add(drive.followBluePath(preloadToStation));
        var stationToScore1 = AutoPaths.loadChoreoTrajectory(
            "Station "
            + getStationPositionAsString(_stationPosition)
            + " To "
            + getBranchLetterFromIndex(_scoreCoral1.getIndex())
        );
        commands.add(drive.followBluePath(stationToScore1));
        var coral1ToStation = AutoPaths.loadChoreoTrajectory(
            getBranchLetterFromIndex(_scoreCoral1.getIndex())+
            " To Station "+
            getStationPositionAsString(_stationPosition)
        );
        commands.add(drive.followBluePath(coral1ToStation));
        var stationToScore2 = AutoPaths.loadChoreoTrajectory(
            "Station "
            + getStationPositionAsString(_stationPosition)
            + " To "
            + getBranchLetterFromIndex(_scoreCoral2.getIndex())
        );
        commands.add(drive.followBluePath(stationToScore2));
        return AutoCommons.setOdometryFlipped(null, drive);
    }

    private static char getBranchLetterFromIndex(int index){
        return (char) (index+'A');
    }
    private String getStationPositionAsString(CoralStationPosition _stationPosition){
        return switch(_stationPosition){
            case CLOSE -> "Close";
            case MID -> "Mid";
            case FAR -> "Far";
            default -> null;};
    }
}
