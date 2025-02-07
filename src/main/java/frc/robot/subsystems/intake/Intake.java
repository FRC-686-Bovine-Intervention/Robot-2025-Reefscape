package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;
import frc.util.robotStructure.GamepiecePose;

public class Intake extends SubsystemBase{
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public static final LoggedTunableMeasure<VoltageUnit> intakeVoltage = new LoggedTunableMeasure<>("Intake/Voltages/Intake", Volts.of(4));
    public static final LoggedTunableMeasure<VoltageUnit> ejectVoltage = new LoggedTunableMeasure<>("Intake/Voltages/Eject", Volts.of(4).unaryMinus());
    public static final LoggedTunableMeasure<VoltageUnit> holdVoltage = new LoggedTunableMeasure<>("Intake/Voltages/Hold", Volts.of(1));

    public final GamepiecePose coralPose = new GamepiecePose(IntakeConstants.coralPose);
    public final GamepiecePose algaePose = new GamepiecePose(IntakeConstants.algaePose);

    public final Trigger hasCoral = new Trigger(() -> inputs.coralSensor);
    public final Trigger hasAlgae = new Trigger(() -> inputs.algaeSensor);

    public Intake(IntakeIO io) {
        System.out.println("[Init Intake] Instantiated Intake with " + io.getClass().getSimpleName());
        this.io = io;
        SmartDashboard.putData("Subsystems/Intake", this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Intake", inputs);

        Logger.recordOutput("Gamepiece/Coral",
            (hasCoral.getAsBoolean()) ? (
                new Pose3d[]{
                    coralPose.getFieldRelative()
                }
            ) : (
                new Pose3d[]{}
            )
        );
        Logger.recordOutput("Gamepiece/Algae",
            (hasAlgae.getAsBoolean()) ? (
                new Pose3d[]{
                    algaePose.getFieldRelative()
                }
            ) : (
                new Pose3d[]{}
            )
        );
    }

    private Command genCommand(
        String name,
        Supplier<Measure<VoltageUnit>> voltage
    ) {
        var subsystem = this;
        return new Command() {
            {
                setName(name);
                addRequirements(subsystem);
            }

            @Override
            public void initialize() {

            }

            @Override
            public void execute() {
                io.setMotorVoltage(voltage.get());
            }

            @Override
            public void end(boolean interrupted) {
                io.setMotorVoltage(Volts.zero());
            }
        };
    }

    public Command stop(){
        return genCommand(
            "Stop", 
            Volts::zero
        );
    }

    public Command idle() {
        return genCommand(
            "Idle",
            holdVoltage
        );
    }

    public Command eject() {
        return genCommand(
            "Eject",
            ejectVoltage
        );
    }

    public Command intake(){
        return genCommand(
            "Intake",
            intakeVoltage          
        );
    }
}