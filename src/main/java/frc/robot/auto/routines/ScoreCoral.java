package frc.robot.auto.routines;

import static frc.robot.auto.AutoCommons.pipeOptions;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.IntFunction;
import java.util.stream.IntStream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoCommons.CoralStationPosition;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoRoutine;
import frc.robot.constants.FieldConstants.Reef.BranchLevel;
import frc.robot.constants.FieldConstants.Reef.PipeObject;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Direction;
import frc.util.flipping.AllianceFlipped;
import frc.util.misc.MathExtraUtil;

public class ScoreCoral extends AutoRoutine {
    // scoring preload (reef pipes)
    // starting position (closest (center pillar), far?)
    // scoring coral 1 (1/2 reef pipes - 1)
    // scoring coral 2 (1/2 reef pipes - 2)
    // which part of the coral station (close, mid, far)

    private static final AutoQuestion<AllianceFlipped<Pose2d>> startPosition = new AutoQuestion<AllianceFlipped<Pose2d>>("Starting Position") {
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startLeftLeftCage = Settings.option("LL", AutoConstants.startLeftLeftCage);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startLeftMiddleCage = Settings.option("LM", AutoConstants.startLeftMiddleCage);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startLeftRightCage = Settings.option("LR", AutoConstants.startLeftRightCage);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startDeadCenter = Settings.option("C", AutoConstants.startDeadCenter);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startRightLeftCage = Settings.option("RL", AutoConstants.startRightLeftCage);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startRightMiddleCage = Settings.option("RM", AutoConstants.startRightMiddleCage);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startRightRightCage = Settings.option("RR", AutoConstants.startRightRightCage);

        @Override
        protected Settings<AllianceFlipped<Pose2d>> generateSettings() {
            return Settings.from(startDeadCenter, startLeftLeftCage, startLeftMiddleCage, startLeftRightCage, startDeadCenter, startRightLeftCage, startRightMiddleCage, startRightRightCage);
        }
    };

    private static final AutoQuestion<CoralStationPosition> stationPosition = new AutoQuestion<CoralStationPosition>("Coral Station Position") {
        private static final Map.Entry<String, CoralStationPosition> stationFar = Settings.option("Far", CoralStationPosition.FAR);
        private static final Map.Entry<String, CoralStationPosition> stationMid = Settings.option("Mid", CoralStationPosition.MID);
        private static final Map.Entry<String, CoralStationPosition> stationClose = Settings.option("Close", CoralStationPosition.CLOSE);
        
        @Override
        protected Settings<CoralStationPosition> generateSettings() {
            return Settings.from(stationClose, stationClose, stationMid, stationFar);
        }
    };
    
    private static boolean isRightCoralStation(PipeObject pipe){
        return isRightCoralStation(pipe.id);
    }
    private static boolean isRightCoralStation(int pipeID){
        return MathExtraUtil.isWithin(pipeID, 1, 6);
    }

    private static final AutoQuestion<AllianceFlipped<PipeObject>> scorePreloadPipe = new AutoQuestion<AllianceFlipped<PipeObject>>("Score Preload Pipe") {
        @Override
        protected Settings<AllianceFlipped<PipeObject>> generateSettings() {
            var startPosition = ScoreCoral.startPosition.getResponse();
            if (
                startPosition == AutoConstants.startLeftLeftCage ||
                startPosition == AutoConstants.startLeftMiddleCage
            ) {
                return Settings.from(pipeOptions[10],
                    pipeOptions[0],
                    pipeOptions[10],
                    pipeOptions[11]
                );
            } else if (startPosition == AutoConstants.startLeftRightCage) {
                return Settings.from(pipeOptions[8],
                    pipeOptions[8],
                    pipeOptions[9]
                );
            } else if (startPosition == AutoConstants.startDeadCenter) {
                return Settings.from(pipeOptions[6],
                    pipeOptions[6],
                    pipeOptions[7]
                );
            } else if (startPosition == AutoConstants.startRightLeftCage) {
                return Settings.from(pipeOptions[4],
                    pipeOptions[4],
                    pipeOptions[5]
                );
            } else if (
                startPosition == AutoConstants.startRightMiddleCage ||
                startPosition == AutoConstants.startRightRightCage
            ) {
                return Settings.from(pipeOptions[3],
                    pipeOptions[1],
                    pipeOptions[2],
                    pipeOptions[3]
                );
            } else {
                return null;
            }
        }

    };

    private static final AutoQuestion<AllianceFlipped<PipeObject>> scoreCoral1 = new AutoQuestion<AllianceFlipped<PipeObject>>("Score Second Pipe") {
        @Override
        protected Settings<AllianceFlipped<PipeObject>> generateSettings() {
            if (isRightCoralStation(scorePreloadPipe.getResponse().getOurs())) {
                var options = IntStream.range(1, 7)
                    .mapToObj(i -> pipeOptions[i % pipeOptions.length])
                    .filter(entry -> !entry.getValue().equals(scorePreloadPipe.getResponse()))
                    .sorted((e1, e2) -> e1.getKey().compareTo(e2.getKey()))
                    .toArray((IntFunction<Map.Entry<String, AllianceFlipped<PipeObject>>[]>) Map.Entry[]::new)
                ;
                var defaultOption = IntStream.of(2,1,3)
                    .mapToObj(i -> pipeOptions[i % pipeOptions.length])
                    .filter(entry -> !entry.getValue().equals(scorePreloadPipe.getResponse()))
                    .findFirst()
                    .get()
                ;
                return Settings.from(defaultOption, options);
            } else {
                var options = IntStream.range(7, 13)
                    .mapToObj(i -> pipeOptions[i % pipeOptions.length])
                    .filter(entry -> !entry.getValue().equals(scorePreloadPipe.getResponse()))
                    .sorted((e1, e2) -> e1.getKey().compareTo(e2.getKey()))
                    .toArray((IntFunction<Map.Entry<String, AllianceFlipped<PipeObject>>[]>) Map.Entry[]::new)
                ;
                var defaultOption = IntStream.of(11,0,10)
                    .mapToObj(i -> pipeOptions[i % pipeOptions.length])
                    .filter(entry -> !entry.getValue().equals(scorePreloadPipe.getResponse()))
                    .findFirst()
                    .get()
                ;
                return Settings.from(defaultOption, options);
            }
        }
    };

    private static final AutoQuestion<AllianceFlipped<PipeObject>> scoreCoral2 = new AutoQuestion<AllianceFlipped<PipeObject>>("Score Third Pipe") {
        @Override
        protected Settings<AllianceFlipped<PipeObject>> generateSettings() {
            if (isRightCoralStation(scorePreloadPipe.getResponse().getOurs())) {
                var options = IntStream.range(1, 7)
                    .mapToObj(i -> pipeOptions[i % pipeOptions.length])
                    .filter(entry -> 
                        !entry.getValue().equals(scorePreloadPipe.getResponse()) &&
                        !entry.getValue().equals(scoreCoral1.getResponse())
                    )
                    .sorted((e1, e2) -> e1.getKey().compareTo(e2.getKey()))
                    .toArray((IntFunction<Map.Entry<String, AllianceFlipped<PipeObject>>[]>) Map.Entry[]::new)
                ;
                var defaultOption = IntStream.of(2,1,3)
                    .mapToObj(i -> pipeOptions[i % pipeOptions.length])
                    .filter(entry -> 
                        !entry.getValue().equals(scorePreloadPipe.getResponse()) &&
                        !entry.getValue().equals(scoreCoral1.getResponse())
                    )
                    .findFirst()
                    .get()
                ;
                return Settings.from(defaultOption, options);
            } else {
                var options = IntStream.range(7, 13)
                    .mapToObj(i -> pipeOptions[i % pipeOptions.length])
                    .filter(entry -> 
                        !entry.getValue().equals(scorePreloadPipe.getResponse()) &&
                        !entry.getValue().equals(scoreCoral1.getResponse())
                    )
                    .sorted((e1, e2) -> e1.getKey().compareTo(e2.getKey()))
                    .toArray((IntFunction<Map.Entry<String, AllianceFlipped<PipeObject>>[]>) Map.Entry[]::new)
                ;
                var defaultOption = IntStream.of(11,0,10)
                    .mapToObj(i -> pipeOptions[i % pipeOptions.length])
                    .filter(entry -> 
                        !entry.getValue().equals(scorePreloadPipe.getResponse()) &&
                        !entry.getValue().equals(scoreCoral1.getResponse())
                    )
                    .findFirst()
                    .get()
                ;
                return Settings.from(defaultOption, options);
            }
        }
    };

    private final Drive drive;
    private final Superstructure superstructure;
    private final Intake intake;

    public ScoreCoral(RobotContainer robot) {
        super("Score Coral", List.of(
            startPosition,
            stationPosition,
            scorePreloadPipe,
            scoreCoral1,
            scoreCoral2
        ));
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
            startPosition.equals(AutoConstants.startRightRightCage) ||
            startPosition.equals(AutoConstants.startLeftLeftCage)
        ) {
            startToScorePath = "Remote Start To " + scorePreloadPipe.getOurs().getLetter();
        } else {
            startToScorePath = "Start To " + scorePreloadPipe.getOurs().getLetter();
        }
        var startToScorePreload = AutoPaths.loadChoreoTrajectory(startToScorePath);
        commands.add(AutoCommons.scoreOnReef(startToScorePreload, BranchLevel.Level4, Direction.Forward, drive, superstructure, intake));


        var preloadToStation = AutoPaths.loadChoreoTrajectory(
            scorePreloadPipe.getOurs().getLetter() +
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
            + scoreCoral1.getOurs().getLetter()
        );
        commands.add(AutoCommons.scoreOnReef(stationToScore1, BranchLevel.Level4, Direction.Forward, drive, superstructure, intake));


        var coral1ToStation = AutoPaths.loadChoreoTrajectory(
            scoreCoral1.getOurs().getLetter() +
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
            + scoreCoral2.getOurs().getLetter()
        );
        commands.add(AutoCommons.scoreOnReef(stationToScore2, BranchLevel.Level4, Direction.Forward, drive, superstructure, intake));

        return Commands.parallel(
            AutoCommons.setOdometryFlipped(startPosition, drive),
            Commands.sequence(commands.toArray(Command[]::new))
        );
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
