package frc.util.loggerUtil;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.util.EdgeDetector;

public class LoggedDashboardCommand extends LoggedNetworkInput {
    private final String key;
    private final BooleanPublisher controllablePublisher;
    private final BooleanPublisher parentPublisher;
    private final StringPublisher namePublisher;
    private final StringPublisher typePublisher;
    private final StringPublisher interruptPublisher;
    private final BooleanEntry runningEntry;
    private final BooleanPublisher disabledPublisher;

    private final Command command;

    private final EdgeDetector dashboardEdgeDetector = new EdgeDetector();
    private boolean dashboardToggle;
    private final EdgeDetector toggleEdgeDetector = new EdgeDetector();

    public LoggedDashboardCommand(String key, Command command) {
        this.key = "/SmartDashboard/" + key;
        this.command = command;
        var table = NetworkTableInstance.getDefault().getTable(this.key);
        this.controllablePublisher = table.getBooleanTopic(".controllable").publish();
        this.parentPublisher = table.getBooleanTopic(".isParented").publish();
        this.namePublisher = table.getStringTopic(".name").publish();
        this.typePublisher = table.getStringTopic(".type").publish();
        this.interruptPublisher = table.getStringTopic(".interruptBehavior").publish();
        this.runningEntry = table.getBooleanTopic("running").getEntry(false);
        this.disabledPublisher = table.getBooleanTopic(".runsWhenDisabled").publish();

        controllablePublisher.set(true);
        parentPublisher.set(CommandScheduler.getInstance().isComposed(command));
        namePublisher.set(command.getClass().getSimpleName());
        typePublisher.set("Command");
        interruptPublisher.set(command.getInterruptionBehavior().toString());
        runningEntry.set(false);
        disabledPublisher.set(command.runsWhenDisabled());
        Logger.registerDashboardInput(this);
    }

    @Override
    public void periodic() {
        if (!Logger.hasReplaySource()) {
            var dashboardValue = runningEntry.get();
            dashboardEdgeDetector.update(dashboardValue);
            if (dashboardEdgeDetector.changed()) {
                dashboardToggle = !dashboardToggle;
            }
        }
        Logger.processInputs(prefix, inputs);
        toggleEdgeDetector.update(dashboardToggle);
        if (toggleEdgeDetector.changed()) {
            if (command.isScheduled()) {
                command.cancel();
            } else {
                command.schedule();
            }
        }
        var scheduled = command.isScheduled();
        runningEntry.set(scheduled);
        dashboardEdgeDetector.reset(scheduled);
    }

    private final LoggableInputs inputs = new LoggableInputs() {
        public void toLog(LogTable table) {
            table.put(removeSlash(key), dashboardToggle);
        }

        public void fromLog(LogTable table) {
            dashboardToggle = table.get(removeSlash(key), false);
        }
    };
}
