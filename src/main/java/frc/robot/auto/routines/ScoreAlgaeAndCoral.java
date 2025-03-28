package frc.robot.auto.routines;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoCommons.BargePosition;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoRoutine;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants.Reef.ReefObject.BranchLevel;
import frc.robot.constants.FieldConstants.Reef.ReefObject.PipeObject;
import frc.robot.constants.FieldConstants.Reef.ReefObject.StagedAlgaeObject;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Direction;
import frc.util.flipping.AllianceFlipped;

public class ScoreAlgaeAndCoral extends AutoRoutine{

    //Where to score preload coral
    //Where to score algae
    //Where to pick up second algae
    //Where to score second algae

    private static final AutoQuestion<AllianceFlipped<PipeObject>> scorePreloadPipe = new AutoQuestion<AllianceFlipped<PipeObject>>("Score Preload Pipe") {
        @Override
        protected Settings<AllianceFlipped<PipeObject>> generateSettings() {
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

    private static final AutoQuestion<AllianceFlipped<StagedAlgaeObject>> intakeAlgae2 = new AutoQuestion<AllianceFlipped<StagedAlgaeObject>>("2nd Algae Rack") {
        @Override
        protected Settings<AllianceFlipped<StagedAlgaeObject>> generateSettings() {
            return Settings.from(
                AutoCommons.algaeOptions[2],
                AutoCommons.algaeOptions[2],
                AutoCommons.algaeOptions[3],
                AutoCommons.algaeOptions[4],
                AutoCommons.algaeOptions[5]
            );
        }
    };

    private final Drive drive;
    private final Superstructure superstructure;
    private final Intake intake;

    public ScoreAlgaeAndCoral(RobotContainer robot){
        super("Score Algae And Coral", List.of(scorePreloadPipe, bargePosition, intakeAlgae2));
        this.drive = robot.drive;
        this.superstructure = robot.superstructure;
        this.intake = robot.intake;
    }

    @Override
    public Command generateCommand() {
        var scorePreloadPipe = ScoreAlgaeAndCoral.scorePreloadPipe.getResponse();
        var bargePosition = ScoreAlgaeAndCoral.bargePosition.getResponse();
        var intakeAlgae2 = ScoreAlgaeAndCoral.intakeAlgae2.getResponse();
        var commands = new ArrayList<Command>();
        var startPosition = AutoConstants.startDeadCenter;

        var startToScorePreload = AutoPaths.loadChoreoTrajectory("Start To " + scorePreloadPipe.getOurs().getLetter());
        commands.add(AutoCommons.scoreOnReef(startToScorePreload, BranchLevel.Level4, Direction.Forward, drive, superstructure, intake));

        var preloadToAlgae = AutoPaths.loadChoreoTrajectory(scorePreloadPipe.getOurs().getLetter() + " To " + "3");
        commands.add(Commands.sequence(
            Commands.parallel(
                drive.followBluePath(preloadToAlgae),
                superstructure.goToSetpointSequenced(Reef.reefs.getOurs().stagedAlgae[3].intakeTotalState.getSuperstructureState(Direction.Forward))
            ),
            // Commands.waitSeconds(2)
            intake.intakeAlgae().until(intake.hasAlgae)
        ));
        var algaeToBarge = AutoPaths.loadChoreoTrajectory("3" + " To " + AutoCommons.getBargePositionAsString(bargePosition));
        commands.add(Commands.sequence(
            Commands.parallel(
                drive.followBluePath(algaeToBarge),
                superstructure.goToSetpointSequenced(FieldConstants.Barge.superstructureState.getForward())
            ),
            // Commands.waitSeconds(2)
            intake.eject().until(intake.hasAlgae.negate())
        ));
        var bargeToAlgae = AutoPaths.loadChoreoTrajectory(AutoCommons.getBargePositionAsString(bargePosition) + " To " + intakeAlgae2.getOurs().rack.id);
        commands.add(Commands.sequence(
            Commands.parallel(
                drive.followBluePath(bargeToAlgae),
                superstructure.goToSetpointSequenced(intakeAlgae2.getOurs().intakeTotalState.getSuperstructureState(Direction.Forward))
            ),
            // Commands.waitSeconds(2)
            intake.intakeAlgae().until(intake.hasAlgae)
        ));
        var algaeToBarge1 = AutoPaths.loadChoreoTrajectory(intakeAlgae2.getOurs().rack.id + " To " + AutoCommons.getBargePositionAsString(bargePosition));
        commands.add(Commands.sequence(
            Commands.parallel(
                drive.followBluePath(algaeToBarge1),
                superstructure.goToSetpointSequenced(FieldConstants.Barge.superstructureState.getForward())
            ),
            // Commands.waitSeconds(2)
            intake.eject().until(intake.hasAlgae.negate())
        ));
        return AutoCommons
            .setOdometryFlipped(startPosition, drive)
            .andThen(commands.toArray(Command[]::new));
    }
    
}
