package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.IntFunction;
import java.util.stream.IntStream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoCommons.BargePosition;
import frc.robot.auto.AutoRoutine.AutoQuestion.Settings;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef.Level;
import frc.robot.constants.FieldConstants.Reef.Pipe;
import frc.robot.constants.FieldConstants.Reef.Rack;
import frc.robot.constants.FieldConstants.Reef.StagedAlgae;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.util.flipping.AllianceFlipped;

public class ScoreAlgaeAndCoral extends AutoRoutine{

    //Where to score preload coral
    //Where to score algae
    //Where to pick up second algae
    //Where to score second algae

    private static final AutoQuestion<Pipe> scorePreloadPipe = new AutoQuestion<Pipe>("Score Preload Pipe") {
        @Override
        protected Settings<Pipe> generateSettings() {
            return Settings.from(
                AutoCommons.pipeOptions[6],
                AutoCommons.pipeOptions[6],
                AutoCommons.pipeOptions[7]
            );
        }
    };
    private static final AutoQuestion<BargePosition> bargePosition = new AutoQuestion<BargePosition>("Barge Position") {
        private static final Map.Entry<String, BargePosition> bargeLeft = Settings.option("Left", BargePosition.LEFT);
        private static final Map.Entry<String, BargePosition> bargeCenter = Settings.option("Center", BargePosition.CENTER);
        private static final Map.Entry<String, BargePosition> bargeRight = Settings.option("Right", BargePosition.RIGHT);
        
        @Override
        protected Settings<BargePosition> generateSettings() {
            return Settings.from(bargeRight, bargeRight, bargeCenter, bargeLeft);
        }
    };

    private static final AutoQuestion<Rack> scoreAlgae = new AutoQuestion<Rack>("2nd Algae Rack") {
        @Override
        protected Settings<Rack> generateSettings() {
            return Settings.from(
                AutoCommons.rackOptions[2],
                AutoCommons.rackOptions[2],
                AutoCommons.rackOptions[3],
                AutoCommons.rackOptions[4],
                AutoCommons.rackOptions[5]
            );
        }
    };




    private final Drive drive;
    private final Superstructure superstructure;
    private final Intake intake;

    public ScoreAlgaeAndCoral(RobotContainer robot){
        super("ScoreAlgaeAndCoral", List.of(scorePreloadPipe, bargePosition, scoreAlgae));
        this.drive = robot.drive;
        this.superstructure = robot.superstructure;
        this.intake = robot.intake;
    }

    @Override
    public Command generateCommand() {
        var scorePreloadPipe = ScoreAlgaeAndCoral.scorePreloadPipe.getResponse();
        var bargePosition = ScoreAlgaeAndCoral.bargePosition.getResponse();
        var scoreAlgae = ScoreAlgaeAndCoral.scoreAlgae.getResponse();
        var commands = new ArrayList<Command>();
        var startPosition = scorePreloadPipe.equals(FieldConstants.Reef.pipes[6]) ? AutoConstants.startRightCenter : AutoConstants.startLeftCenter;

        var startToScorePreload = AutoPaths.loadChoreoTrajectory("Start To " + scorePreloadPipe.getLetter());
        commands.add(Commands.sequence(
            Commands.parallel(
                drive.followBluePath(startToScorePreload),
                superstructure.goToSetpointSequenced(Level.Level4.superstructureStates.getForward())
            ),
            intake.eject().until(intake.hasCoral.negate())
        ));
        var preloadToAlgae = AutoPaths.loadChoreoTrajectory(scorePreloadPipe.getLetter() + " To " + scorePreloadPipe.rack.ordinal());
        commands.add(Commands.sequence(
            Commands.parallel(
                drive.followBluePath(preloadToAlgae),
                superstructure.goToSetpointSequenced(scorePreloadPipe.rack.stagedAlgae.algaeLevel.superstructurePosition.getForward())
            ),
            intake.intake().until(intake.hasAlgae)
        ));
        var algaeToBarge = AutoPaths.loadChoreoTrajectory(scorePreloadPipe.rack.ordinal() + " To " + AutoCommons.getBargePositionAsString(bargePosition));
        commands.add(Commands.sequence(
            Commands.parallel(
                drive.followBluePath(algaeToBarge),
                superstructure.goToSetpointSequenced(FieldConstants.Barge.superstructurePosition.getForward())
            ),
            intake.eject().until(intake.hasAlgae.negate())
        ));
        var bargeToAlgae = AutoPaths.loadChoreoTrajectory(AutoCommons.getBargePositionAsString(bargePosition) + " To " + scoreAlgae.ordinal());
        commands.add(Commands.sequence(
            Commands.parallel(
                drive.followBluePath(bargeToAlgae),
                superstructure.goToSetpointSequenced(scoreAlgae.stagedAlgae.algaeLevel.superstructurePosition.getForward())
            ),
            intake.intake().until(intake.hasAlgae)
        ));
        var algaeToBarge1 = AutoPaths.loadChoreoTrajectory(scoreAlgae.ordinal() + " To " + AutoCommons.getBargePositionAsString(bargePosition));
        commands.add(Commands.sequence(
            Commands.parallel(
                drive.followBluePath(algaeToBarge1),
                superstructure.goToSetpointSequenced(FieldConstants.Barge.superstructurePosition.getForward())
            ),
            intake.eject().until(intake.hasAlgae.negate())
        ));
        return AutoCommons
            .setOdometryFlipped(startPosition, drive)
            .andThen(commands.toArray(Command[]::new));
    }
    
}
