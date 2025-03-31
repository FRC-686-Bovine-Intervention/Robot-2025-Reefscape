package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private static final LoggedTunableMeasure<VoltageUnit> idleVoltage = new LoggedTunableMeasure<>("Climber/Idle Voltage", Volts.of(0));
    private static final LoggedTunableMeasure<AngleUnit> ratchetEngageAngle = new LoggedTunableMeasure<>("Climber/Ratchet/Engage Angle", Degrees.of(15));
    private static final LoggedTunableMeasure<AngleUnit> ratchetDisengageAngle = new LoggedTunableMeasure<>("Climber/Ratchet/Disengage Angle", Degrees.of(65));
    private static final LoggedTunableMeasure<AngleUnit> deployAngle = new LoggedTunableMeasure<>("Climber/Deploy Angle", Rotations.of(4));
    private static final LoggedTunableMeasure<AngleUnit> climbAngle = new LoggedTunableMeasure<>("Climber/Climb Angle", Rotations.of(1.5));
    private static final LoggedTunableMeasure<AngleUnit> climbTolerance = new LoggedTunableMeasure<>("Climber/Climb Tolerance", Rotations.of(0.05));
    private static final LoggedTunableMeasure<TimeUnit> ratchetTime = new LoggedTunableMeasure<>("Climber/Ratchet Time", Seconds.of(0.25));

    private boolean ratchetEngaged = true;

    public Climber(ClimberIO io) {
        System.out.println("[Init Climber] Instantiating Climber with " + io.getClass().getSimpleName());
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Climber", inputs);
        Logger.recordOutput("Climber/Position", getAngle());
        Logger.recordOutput("Climber/Ratchet Engaged", ratchetEngaged);
    }

    public Angle getAngle(){
        return ClimberConstants.sensorToMechanismRatio.apply(inputs.motor.encoder.position).unaryMinus();
    }

    public Command idle() {
        var subsystem = this;
        return new Command() {
            private final Timer ratchetTimer = new Timer();
            {
                addRequirements(subsystem);
                setName("Idle");
            }
            @Override
            public void initialize() {               
            }

            @Override
            public void execute() {
                io.setRatchetServoAngle(ratchetDisengageAngle.get());
                if (ratchetEngaged) {
                    io.setVoltage(Volts.zero());
                    ratchetTimer.start();
                    if(ratchetTimer.hasElapsed(ratchetTime.get().in(Seconds))){
                        ratchetEngaged = false;
                    }
                } else {
                    ratchetTimer.stop();
                    ratchetTimer.reset();
                    io.setVoltage(idleVoltage.get());
                }
            }
            @Override
            public void end(boolean interrupted) {
                
            }
        };
    }
    
    public Command prepareClimb() {
        var subsystem = this;
        return new Command() {
            private final Timer ratchetTimer = new Timer();
            {
                addRequirements(subsystem);
                setName("Prepare Climb");
            }
            @Override
            public void initialize() {               
            }

            @Override
            public void execute() {
                io.setRatchetServoAngle(ratchetDisengageAngle.get());
                if(ratchetEngaged){
                    ratchetTimer.start();
                    if(ratchetTimer.hasElapsed(ratchetTime.get().in(Seconds))){
                        ratchetEngaged = false;
                    }
                } else {
                    ratchetTimer.stop();
                    ratchetTimer.reset();
                    io.setAngle(deployAngle.get());
                }
            }
            @Override
            public void end(boolean interrupted) {
                
            }
        };
    }

    public Command climb() {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("Climb");
            }
            @Override
            public void initialize() {   

            }

            @Override
            public void execute() {
                io.setRatchetServoAngle(ratchetDisengageAngle.get());
                ratchetEngaged = true;
                io.setAngle(climbAngle.get());
            }
            @Override
            public void end(boolean interrupted) {
                
            }
        };
    }

    public Command engageRatchet() {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("Engage Ratchet");
            }
            @Override
            public void initialize() {   

            }

            @Override
            public void execute() {
                io.setRatchetServoAngle(ratchetEngageAngle.get());
                ratchetEngaged = true;
                io.setVoltage(Volts.zero());
            }
            @Override
            public void end(boolean interrupted) {
                
            }
        };
    }
    public Command disengageRatchet() {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("Disengage Ratchet");
            }
            @Override
            public void initialize() {   

            }

            @Override
            public void execute() {
                io.setRatchetServoAngle(ratchetDisengageAngle.get());
                ratchetEngaged = false;
                io.setVoltage(Volts.zero());
            }
            @Override
            public void end(boolean interrupted) {
                
            }
        };
    }
}