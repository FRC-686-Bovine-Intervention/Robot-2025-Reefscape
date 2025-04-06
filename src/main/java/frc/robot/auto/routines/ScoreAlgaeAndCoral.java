package frc.robot.auto.routines;

import static frc.robot.auto.AutoCommons.getStartingPositionAsString;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.IntFunction;
import java.util.function.Predicate;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoCommons.BargePosition;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoRoutine;
import frc.robot.constants.FieldConstants.Reef.BranchLevel;
import frc.robot.constants.FieldConstants.Reef.PipeConcept;
import frc.robot.constants.FieldConstants.Reef.StagedAlgaeConcept;
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

    private static final AutoQuestion<PipeConcept> firstCoralPipe = new AutoQuestion<PipeConcept>("Score Preload Pipe") {
        @Override
        protected Settings<PipeConcept> generateSettings() {
            var startPosition = ScoreAlgaeAndCoral.startPosition.getResponse();
            if (
                startPosition == AutoConstants.startLeftLeftCage ||
                startPosition == AutoConstants.startLeftMiddleCage
            ) {
                return Settings.from(AutoCommons.pipeOptions[9],
                    // AutoCommons.pipeOptions[6],
                    // AutoCommons.pipeOptions[7],
                    AutoCommons.pipeOptions[8],
                    AutoCommons.pipeOptions[9]
                );
            } else if (startPosition == AutoConstants.startLeftRightCage) {
                return Settings.from(AutoCommons.pipeOptions[9],
                    AutoCommons.pipeOptions[6],
                    AutoCommons.pipeOptions[7],
                    AutoCommons.pipeOptions[8],
                    AutoCommons.pipeOptions[9]
                );
            } else if (startPosition == AutoConstants.startDeadCenter) {
                return Settings.from(AutoCommons.pipeOptions[6],
                    AutoCommons.pipeOptions[4],
                    AutoCommons.pipeOptions[5],
                    AutoCommons.pipeOptions[6],
                    AutoCommons.pipeOptions[7],
                    AutoCommons.pipeOptions[8],
                    AutoCommons.pipeOptions[9]
                );
            } else if (startPosition == AutoConstants.startRightLeftCage) {
                return Settings.from(AutoCommons.pipeOptions[4],
                    AutoCommons.pipeOptions[4],
                    AutoCommons.pipeOptions[5],
                    AutoCommons.pipeOptions[6],
                    AutoCommons.pipeOptions[7]
                );
            } else if (
                startPosition == AutoConstants.startRightMiddleCage ||
                startPosition == AutoConstants.startRightRightCage
            ) {
                return Settings.from(AutoCommons.pipeOptions[4],
                    AutoCommons.pipeOptions[4],
                    AutoCommons.pipeOptions[5]//,
                    // AutoCommons.pipeOptions[6],
                    // AutoCommons.pipeOptions[7]
                );
            } else {
                return null;
            }
        }
    };
    private static final AutoQuestion<BargePosition> bargePosition = new AutoQuestion<BargePosition>("Barge Position") {
        private static final Map.Entry<String, BargePosition> bargeLeft = Settings.option("Left", BargePosition.LEFT);
        private static final Map.Entry<String, BargePosition> bargeCenter = Settings.option("Center", BargePosition.CENTER);
        private static final Map.Entry<String, BargePosition> bargeRight = Settings.option("Right", BargePosition.RIGHT);
        
        @Override
        protected Settings<BargePosition> generateSettings() {
            return Settings.from(bargeRight, bargeLeft, bargeCenter, bargeRight);
        }
    };

    private static final AutoQuestion<StagedAlgaeConcept> firstAlgae = new AutoQuestion<StagedAlgaeConcept>("First Algae Rack") {
        @Override
        protected Settings<StagedAlgaeConcept> generateSettings() {
            if (firstCoralPipe.getResponse().id == 4 || firstCoralPipe.getResponse().id == 5) {
                return Settings.from(AutoCommons.algaeOptions[2],
                    AutoCommons.algaeOptions[2],
                    AutoCommons.algaeOptions[3]
                );
            } else if (firstCoralPipe.getResponse().id == 6 || firstCoralPipe.getResponse().id == 7) {
                return Settings.from(AutoCommons.algaeOptions[3],
                    AutoCommons.algaeOptions[2],
                    AutoCommons.algaeOptions[3],
                    AutoCommons.algaeOptions[4]
                );
            } else {
                return Settings.from(AutoCommons.algaeOptions[4],
                    AutoCommons.algaeOptions[3],
                    AutoCommons.algaeOptions[4]
                );
            }
        }
    };

    private static final AutoQuestion<Optional<StagedAlgaeConcept>> secondAlgae = new AutoQuestion<Optional<StagedAlgaeConcept>>("Second Algae Rack") {
        @Override
        protected Settings<Optional<StagedAlgaeConcept>> generateSettings() {
            Predicate<Map.Entry<String, Optional<StagedAlgaeConcept>>> notInPreviousResponses = (option) -> option.getValue().isEmpty() || !option.getValue().equals(Optional.of(firstAlgae.getResponse()));
            var options = Stream.of(
                AutoCommons.algaeOptionalOptions[6],
                AutoCommons.algaeOptionalOptions[2],
                AutoCommons.algaeOptionalOptions[3],
                AutoCommons.algaeOptionalOptions[4]
            )
                .filter(notInPreviousResponses)
                .toArray((IntFunction<Map.Entry<String, Optional<StagedAlgaeConcept>>[]>) Map.Entry[]::new)
            ;
            var defaultOption = Stream.of(
                AutoCommons.algaeOptionalOptions[3],
                AutoCommons.algaeOptionalOptions[4],
                AutoCommons.algaeOptionalOptions[2]
            )
                .filter(notInPreviousResponses)
                .findFirst()
                .get()
            ;
            return Settings.from(defaultOption, options);
        }
    };

    private static final AutoQuestion<Optional<StagedAlgaeConcept>> thirdAlgae = new AutoQuestion<Optional<StagedAlgaeConcept>>("Third Algae Rack") {
        @Override
        protected Settings<Optional<StagedAlgaeConcept>> generateSettings() {
            if (secondAlgae.getResponse().isEmpty()) {
                return Settings.from(AutoCommons.algaeOptionalOptions[6], AutoCommons.algaeOptionalOptions[6]);
            }
            Predicate<Map.Entry<String, Optional<StagedAlgaeConcept>>> notInPreviousResponses = (option) -> option.getValue().isEmpty() || (!option.getValue().equals(Optional.of(firstAlgae.getResponse())) && !option.getValue().equals(secondAlgae.getResponse()));
            var options = Stream.of(
                AutoCommons.algaeOptionalOptions[6],
                AutoCommons.algaeOptionalOptions[2],
                AutoCommons.algaeOptionalOptions[3],
                AutoCommons.algaeOptionalOptions[4]
            )
                .filter(notInPreviousResponses)
                .toArray((IntFunction<Map.Entry<String, Optional<StagedAlgaeConcept>>[]>) Map.Entry[]::new)
            ;
            var defaultOption = Stream.of(
                AutoCommons.algaeOptionalOptions[3],
                AutoCommons.algaeOptionalOptions[4],
                AutoCommons.algaeOptionalOptions[2]
            )
                .filter(notInPreviousResponses)
                .findFirst()
                .get()
            ;
            return Settings.from(defaultOption, options);
        }
    };

    private final Drive drive;
    private final Superstructure superstructure;
    private final Intake intake;

    public ScoreAlgaeAndCoral(RobotContainer robot){
        super("Score Algae And Coral", List.of(
            startPosition,
            bargePosition,
            firstCoralPipe,
            firstAlgae,
            secondAlgae,
            thirdAlgae
        ));
        this.drive = robot.drive;
        this.superstructure = robot.superstructure;
        this.intake = robot.intake;
    }

    @Override
    public Command generateCommand() {
        var startPosition = ScoreAlgaeAndCoral.startPosition.getResponse();
        var bargePosition = ScoreAlgaeAndCoral.bargePosition.getResponse();
        var firstCoralPipe = ScoreAlgaeAndCoral.firstCoralPipe.getResponse();
        var firstAlgae = ScoreAlgaeAndCoral.firstAlgae.getResponse();
        var secondAlgae = ScoreAlgaeAndCoral.secondAlgae.getResponse();
        var thirdAlgae = ScoreAlgaeAndCoral.thirdAlgae.getResponse();
        var commands = new ArrayList<Command>();

        commands.add(Commands.waitSeconds(1));

        var startToFirstPipe = AutoPaths.loadChoreoTrajectory(getStartingPositionAsString(startPosition) + " To " + firstCoralPipe.getLetter());
        commands.add(AutoCommons.scoreOnReef(startToFirstPipe, firstCoralPipe.getBranch(BranchLevel.Level4), Direction.Forward, drive, superstructure, intake));

        var firstPipeToFistAlgae = AutoPaths.loadChoreoTrajectory(firstCoralPipe.getLetter() + " To " + firstAlgae.rack.id);
        commands.add(AutoCommons.pickupAlgaeFromReef(firstPipeToFistAlgae, firstAlgae, Direction.Forward, drive, superstructure, intake));

        var firstAlgaeToNet = AutoPaths.loadChoreoTrajectory(firstAlgae.rack.id + " To " + AutoCommons.getBargePositionAsString(bargePosition));
        commands.add(AutoCommons.scoreInNet(firstAlgaeToNet, Direction.Backward, drive, superstructure, intake));

        if (secondAlgae.isPresent()) {
            var algae2 = secondAlgae.get();

            var netToSecondAlgae = AutoPaths.loadChoreoTrajectory(AutoCommons.getBargePositionAsString(bargePosition) + " To " + algae2.rack.id);
            commands.add(AutoCommons.pickupAlgaeFromReef(netToSecondAlgae, algae2, Direction.Forward, drive, superstructure, intake));

            var secondAlgaeToNet = AutoPaths.loadChoreoTrajectory(algae2.rack.id + " To " + AutoCommons.getBargePositionAsString(bargePosition));
            commands.add(AutoCommons.scoreInNet(secondAlgaeToNet, Direction.Backward, drive, superstructure, intake));

            if (thirdAlgae.isPresent()) {
                var algae3 = thirdAlgae.get();
    
                var netToThirdAlgae = AutoPaths.loadChoreoTrajectory(AutoCommons.getBargePositionAsString(bargePosition) + " To " + algae3.rack.id);
                commands.add(AutoCommons.pickupAlgaeFromReef(netToThirdAlgae, algae3, Direction.Forward, drive, superstructure, intake));
    
                var thirdAlgaeToNet = AutoPaths.loadChoreoTrajectory(algae3.rack.id + " To " + AutoCommons.getBargePositionAsString(bargePosition));
                commands.add(AutoCommons.scoreInNet(thirdAlgaeToNet, Direction.Backward, drive, superstructure, intake));
            }
        }

        return Commands.parallel(
            AutoCommons.setOdometryFlipped(startPosition, drive),
            Commands.sequence(commands.toArray(Command[]::new))
        );
    }
    
}
