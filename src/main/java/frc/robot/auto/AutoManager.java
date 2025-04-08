package frc.robot.auto;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GameState;
import frc.robot.subsystems.leds.Leds;
import frc.util.VirtualSubsystem;

public class AutoManager extends VirtualSubsystem {
    private final AutoSelector selector;

    private Command autonomousCommand;
    // private final SuppliedEdgeDetector autoEnabled = new SuppliedEdgeDetector(DriverStation::isAutonomousEnabled);

    public AutoManager(AutoSelector selector) {
        this.selector = selector;
    }

    @Override
    public void periodic() {
        // autoEnabled.update();
        // if(autoEnabled.risingEdge()) {
        //     autonomousCommand = selector.getSelectedAutoCommand();
        //     if(autonomousCommand != null) {
        //         autonomousCommand.schedule();
        //     }
        // }
        // if(autoEnabled.fallingEdge() && autonomousCommand != null) {
        //     autonomousCommand.cancel();
        // }
    }

    public void startAuto() {
        autonomousCommand = selector.getSelectedAutoCommand();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    public void endAuto() {
        if(autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    public static Command generateAutoCommand(AutoRoutine auto, double initialDelaySeconds) {
        final var autoCommand = auto.generateCommand();
        return new Command() {
            private final Timer autoTimer = new Timer();
            {
                setName("AUTO " + auto.name);
                addRequirements(autoCommand.getRequirements());
            }
            private boolean autoCommandRunning = false;

            private void initializeOrExecute() {
                if (autoCommandRunning) {
                    autoCommand.execute();
                } else if (autoTimer.hasElapsed(initialDelaySeconds)) {
                    autoCommand.initialize();
                    autoCommandRunning = true;
                }
            }

            @Override
            public void initialize() {
                autoCommandRunning = false;
                autoTimer.start();
                initializeOrExecute();
                GameState.getInstance().AUTONOMOUS_COMMAND_FINISH.clear();
                GameState.getInstance().AUTONOMOUS_ALLOTTED_TIMESTAMP.set(Timer.getTimestamp() + AutoConstants.allottedAutoTime.in(Seconds));
                Leds.getInstance().autonomousRunningAnimation.setFlag(true);
            }
            @Override
            public void execute() {
                initializeOrExecute();
            }
            @Override
            public void end(boolean interrupted) {
                autoCommand.end(interrupted);
                autoTimer.stop();
                autoTimer.reset();
                Leds.getInstance().autonomousRunningAnimation.setFlag(false);
                GameState.getInstance().AUTONOMOUS_COMMAND_FINISH.set();
                var autoTime = GameState.getInstance().BEGIN_ENABLE.getTimeSince();
                var overrun = autoTime > AutoConstants.allottedAutoTime.in(Seconds);
                Leds.getInstance().autonomousFinishedAnimation.setOverrun(overrun);
                if (overrun) {
                    System.out.println(String.format("[AutoManager] Autonomous overran the allotted %.1f seconds!", AutoConstants.allottedAutoTime.in(Seconds)));
                }
                if (interrupted) {
                    System.out.println(String.format("[AutoManager] Autonomous interrupted after %.2f seconds", autoTime));
                } else {
                    System.out.println(String.format("[AutoManager] Autonomous finished in %.2f seconds", autoTime));
                }
                Leds.getInstance().autonomousFinishedAnimation.setFlagCommand()
                    .until(() -> (overrun) ? (
                        GameState.getInstance().AUTONOMOUS_COMMAND_FINISH.hasBeenSince(3)
                    ) : (
                        GameState.getInstance().AUTONOMOUS_ALLOTTED_TIMESTAMP.hasBeenSince(0)
                    ))
                    .withName("Autonomous LED Notif")
                    .schedule()
                ;
            }
            @Override
            public boolean isFinished() {
                return autoCommand.isFinished();
            }
        };
    }
}
