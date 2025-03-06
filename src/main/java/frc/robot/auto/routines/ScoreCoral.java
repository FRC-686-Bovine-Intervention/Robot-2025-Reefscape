package frc.robot.auto.routines;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.function.IntFunction;
import java.util.stream.IntStream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoRoutine;
import frc.robot.auto.AutoRoutine.AutoQuestion.Settings;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef.Level;
import frc.robot.constants.FieldConstants.Reef.Pipe;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.RobotFlippedSuperstructureState.Direction;
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
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startDeadCenter = Settings.option("Dead Center", AutoConstants.startDeadCenter);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startBlueCageMiddle = Settings.option("Middle Blue Cage", AutoConstants.startBlueCageMiddle);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startBlueCageInner = Settings.option("Inner Blue Cage", AutoConstants.startBlueCageInner);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startBlueCageOuter = Settings.option("Outer Blue Cage", AutoConstants.startBlueCageOuter);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startRedCageMiddle = Settings.option("Middle Red Cage", AutoConstants.startRedCageMiddle);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startRedCageInner = Settings.option("Inner Red Cage", AutoConstants.startRedCageInner);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startRedCageOuter = Settings.option("Outer Red Cage", AutoConstants.startRedCageOuter);


        @Override
        protected Settings<AllianceFlipped<Pose2d>> generateSettings() {
            switch(scorePreloadPipe.getResponse().getIndex()){
                case 0:
                return Settings.from(startBlueCageMiddle, startBlueCageMiddle, startBlueCageOuter);
                case 1:
                return Settings.from(startRedCageMiddle, startRedCageMiddle, startRedCageOuter);
                case 2:
                return Settings.from(startRedCageMiddle, startRedCageMiddle, startRedCageOuter);
                case 3:
                return Settings.from(startRedCageMiddle, startRedCageMiddle, startRedCageOuter);
                case 4:
                return Settings.from(startRedCageInner, startRedCageInner);
                case 5:
                return Settings.from(startRedCageInner, startBlueCageInner);
                case 6:
                return Settings.from(startDeadCenter, startDeadCenter);
                case 7:
                return Settings.from(startDeadCenter, startDeadCenter);
                case 8:
                return Settings.from(startBlueCageInner, startBlueCageInner);
                case 9:
                return Settings.from(startBlueCageInner, startBlueCageInner);
                case 10:
                return Settings.from(startBlueCageMiddle, startBlueCageMiddle, startBlueCageOuter);
                case 11:
                return Settings.from(startBlueCageMiddle, startBlueCageMiddle, startBlueCageOuter);
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
        boolean shouldUseForwardCoralStation = false;

        String startToScorePath;
        if (
            startPosition.equals(AutoConstants.startRedCageOuter) ||
            startPosition.equals(AutoConstants.startBlueCageOuter)
        ) {
            startToScorePath = "Remote Start To " + getBranchLetterFromIndex(scorePreloadPipe.getIndex());
        } else {
            startToScorePath = "Start To " + getBranchLetterFromIndex(scorePreloadPipe.getIndex());
        }
        var startToScorePreload = AutoPaths.loadChoreoTrajectory(startToScorePath);
        commands.add(AutoCommons.scoreOnReef(startToScorePreload, Level.Level4, Direction.Forward, drive, superstructure, intake));


        var preloadToStation = AutoPaths.loadChoreoTrajectory(
            getBranchLetterFromIndex(scorePreloadPipe.getIndex()) +
            " To Station " +
            getStationPositionAsString(stationPosition) +
            (shouldUseForwardCoralStation ? " Forward" : "")
        );
        commands.add(AutoCommons.pickupCoralFromStation(preloadToStation, Direction.Backward, drive, superstructure, intake));


        var stationToScore1 = AutoPaths.loadChoreoTrajectory(
            "Station "
            + getStationPositionAsString(stationPosition)
            + (shouldUseForwardCoralStation ? " Forward" : "")
            + " To "
            + getBranchLetterFromIndex(scoreCoral1.getIndex())
        );
        commands.add(AutoCommons.scoreOnReef(stationToScore1, Level.Level4, Direction.Forward, drive, superstructure, intake));


        var coral1ToStation = AutoPaths.loadChoreoTrajectory(
            getBranchLetterFromIndex(scoreCoral1.getIndex())+
            " To Station "+
            getStationPositionAsString(stationPosition)
            + (shouldUseForwardCoralStation ? " Forward" : "")
        );
        commands.add(AutoCommons.pickupCoralFromStation(coral1ToStation, Direction.Backward, drive, superstructure, intake));
        

        var stationToScore2 = AutoPaths.loadChoreoTrajectory(
            "Station "
            + getStationPositionAsString(stationPosition)
            + (shouldUseForwardCoralStation ? " Forward" : "")
            + " To "
            + getBranchLetterFromIndex(scoreCoral2.getIndex())
        );
        commands.add(AutoCommons.scoreOnReef(stationToScore2, Level.Level4, Direction.Forward, drive, superstructure, intake));

        return
            AutoCommons.setOdometryFlipped(startPosition, drive)
            .andThen(commands.toArray(Command[]::new))
        ;
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
