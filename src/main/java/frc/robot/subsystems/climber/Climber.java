package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private static final LoggedTunableMeasure<VoltageUnit> prepareVoltage = new LoggedTunableMeasure<>("Climber/Prepare/Volts", Volts.of(0.5));
    private static final LoggedTunableMeasure<CurrentUnit> prepareCurrent = new LoggedTunableMeasure<>("Climber/Prepare/Current", Amps.of(50));
    private static final LoggedTunableMeasure<VoltageUnit> climbVoltage = new LoggedTunableMeasure<>("Climber/Climb/Volts", Volts.of(1));
    private static final LoggedTunableMeasure<VoltageUnit> holdVoltage = new LoggedTunableMeasure<>("Climber/Hold/Volts", Volts.of(1));
    private static final LoggedTunableMeasure<VoltageUnit> idleVoltage = new LoggedTunableMeasure<>("Climber/Idle/Volts", Volts.of(-0.1));

    public Climber(ClimberIO io) {
        System.out.println("[Init Climber] Instantiating Climber with " + io.getClass().getSimpleName());
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Climber", inputs);
    }
    
    public Command prepareClimb() {
        var subsystem = this;
        return new Command() {
            private boolean hitSpike = false;
            private final Timer timer = new Timer();
            {
                addRequirements(subsystem);
                setName("Prepare Climb");
            }
            @Override
            public void initialize() {
                hitSpike = false;                
            }
            @Override
            public void execute() {
                if (hitSpike) {
                    io.setCoastVoltage(Volts.of(0));
                } else {
                    if (inputs.motor.motor.current.gt(prepareCurrent.get())) {
                        timer.start();
                    } else {
                        timer.stop();
                        timer.reset();
                    }
                    if (timer.hasElapsed(0.2)) {
                        hitSpike = true;
                    }
                    io.setCoastVoltage(prepareVoltage.get());
                }
            }
            @Override
            public void end(boolean interrupted) {
                io.setCoastVoltage(Volts.of(0));
            }
        };
    }
}