package frc.util.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class InstantSequentialCommandGroup extends Command {
    private final List<Command> commands = new ArrayList<>();
    private int currentCommandIndex = -1;
    private boolean runWhenDisabled = true;
    private InterruptionBehavior interruptBehavior = InterruptionBehavior.kCancelIncoming;

    public InstantSequentialCommandGroup(Command... commands) {
        this.addCommands(commands);
    }

    public void addCommands(Command... commands) {
        if (this.currentCommandIndex != -1) {
            throw new IllegalStateException("Commands cannot be added to a composition while it's running");
        }

        CommandScheduler.getInstance().registerComposedCommands(commands);

        for (Command command : commands) {
            this.commands.add(command);
            this.addRequirements(command.getRequirements());
            this.runWhenDisabled &= command.runsWhenDisabled();
            if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
                this.interruptBehavior = InterruptionBehavior.kCancelSelf;
            }
        }
    }

    @Override
    public void initialize() {
        for (this.currentCommandIndex = 0; this.currentCommandIndex < this.commands.size(); this.currentCommandIndex++) {
            var runningCommand = this.commands.get(this.currentCommandIndex);
            runningCommand.initialize();
            if (runningCommand.isFinished()) {
                runningCommand.end(false);
            } else {
                break;
            }
        }
    }

    @Override
    public void execute() {
        for (; this.currentCommandIndex < this.commands.size(); this.currentCommandIndex++) {
            var runningCommand = this.commands.get(this.currentCommandIndex);
            runningCommand.execute();
            if (runningCommand.isFinished()) {
                runningCommand.end(false);
            } else {
                break;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (
            interrupted
            && this.currentCommandIndex > -1
            && this.currentCommandIndex < this.commands.size()
        ) {
            this.commands.get(this.currentCommandIndex).end(true);
        }
        this.currentCommandIndex = -1;
    }

    @Override
    public boolean isFinished() {
        return this.currentCommandIndex >= this.commands.size();
    }

    @Override
    public boolean runsWhenDisabled() {
        return this.runWhenDisabled;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return this.interruptBehavior;
    }
}
