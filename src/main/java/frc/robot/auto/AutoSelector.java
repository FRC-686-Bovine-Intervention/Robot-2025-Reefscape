package frc.robot.auto;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.util.SwitchableChooser;
import frc.util.VirtualSubsystem;
import frc.util.rust.iter.Iterator;

public class AutoSelector extends VirtualSubsystem {
    private final LoggedDashboardChooser<AutoRoutine> routineChooser;
    private final List<StringPublisher> questionPublishers;
    private final List<SwitchableChooser> responseChoosers;
    private final StringPublisher configPublisher;
    private final String key;
    
    private static final AutoRoutine idleRoutine = new AutoRoutine("Do Nothing", List.of()) {
        public Command generateCommand() {
            return Commands.idle();
        }
    };
    private final String questionPlaceHolder = "NA";

    private Command lastCommand;
    private AutoConfiguration lastConfiguration;

    public AutoSelector(String key) {
        this.key = key;
        routineChooser = new LoggedDashboardChooser<>(key + "/Routine");
        questionPublishers = new ArrayList<>();
        responseChoosers = new ArrayList<>();
        configPublisher = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(key).getStringTopic("Configuration").publish();
        addDefaultRoutine(idleRoutine);
    }

    private void populateQuestions(AutoRoutine routine) {
        for (int i = questionPublishers.size(); i < routine.questions.size(); i++) {
            var questionPublisher = NetworkTableInstance.getDefault()
                .getStringTopic("/SmartDashboard/" + key + "/Question #" + Integer.toString(i + 1))
                .publish()
            ;
            questionPublisher.set(questionPlaceHolder);
            questionPublishers.add(questionPublisher);
            responseChoosers.add(new SwitchableChooser(key + "/Question #" + Integer.toString(i + 1) + " Chooser"));
        }
    }

    public void addRoutine(AutoRoutine routine) {
        populateQuestions(routine);
        routineChooser.addOption(routine.name, routine);
    }

    public void addDefaultRoutine(AutoRoutine routine) {
        populateQuestions(routine);
        routineChooser.addDefaultOption(routine.name, routine);

        doThingy(routine, true);
    }

    @Override
    public void periodic() {
        var selectedRoutine = routineChooser.get();
        if(selectedRoutine == null) return;

        doThingy(
            selectedRoutine,
            selectedRoutine.name != lastConfiguration.routine()
            || DriverStation.getAlliance().orElse(Alliance.Blue) != lastConfiguration.alliance()
        );
    }

    private void doThingy(AutoRoutine routine, boolean configurationChanged) {
        var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        var config = new AutoConfiguration(alliance, routine.name);

        var questions = routine.questions;
        for (int i = 0; i < responseChoosers.size(); i++) {
            var questionPublisher = questionPublishers.get(i);
            var responseChooser = responseChoosers.get(i);

            if (i < questions.size()) {
                var question = questions.get(i);
                questionPublisher.set(question.name);
                
                if (configurationChanged) {
                    var settings = question.updateSettings();
                    var defaultOption = settings.defaultOption().getKey();
                    responseChooser.setOptions(settings.getOptionNames());
                    responseChooser.setDefault(defaultOption);
                    if (Iterator.of(settings.getOptionNames()).all((option) -> responseChooser.getSelected() != option)) {
                        responseChooser.setSelected(defaultOption);
                    }
                } else {
                    var response = responseChooser.getSelected();
                    var last = lastConfiguration.questions().get(question.name);
                    configurationChanged = !Objects.equals(response, last);
                }

                var selectedResponse = responseChooser.getSelected();
                responseChooser.setActive(question.setResponse(selectedResponse));
                config.addQuestion(question.name, responseChooser.getActive());
            } else {
                questionPublisher.set("");
                responseChooser.setOptions();
                responseChooser.setDefault(questionPlaceHolder);
                responseChooser.setSelected(questionPlaceHolder);
                responseChooser.setActive(questionPlaceHolder);
            }
        }
        if (configurationChanged) {
            System.out.println("[AutoSelector] Generating new command\n" + config);
            lastCommand = AutoManager.generateAutoCommand(routine);
        }
        lastConfiguration = config;
        configPublisher.set(lastConfiguration.toString());
    }

    public Command getSelectedAutoCommand() {
        return lastCommand;
    }

    public static record AutoConfiguration (
        Alliance alliance,
        String routine,
        Map<String, String> questions
    ) {
        public AutoConfiguration(Alliance alliance, String routine) {
            this(alliance, routine, new LinkedHashMap<>());
        }
        public void addQuestion(String question, String response) {
            questions.put(question, response);
        }

        public String toString() {
            var builder = new StringBuilder()
                .append("\t").append("Alliance: ").append(alliance).append("\n")
                .append("\t").append("Routine: ").append(routine)
            ;
            for (var entry : questions.entrySet()) {
                builder.append("\n\t").append(entry.getKey()).append(": ").append(entry.getValue());
            }
            return builder.toString();
        }
    }
}
