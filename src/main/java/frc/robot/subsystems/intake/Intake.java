package frc.robot.subsystems.intake;

import java.util.function.Supplier;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;
import frc.util.robotStructure.GamepiecePose;

public class Intake extends SubsystemBase{
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    //These two need to be set
    public static final LoggedTunableMeasure<VoltageUnit> intakeVoltage = new LoggedTunableMeasure<>("Intake/Voltages/Intake", Volts.of(4));
    public static final LoggedTunableMeasure<VoltageUnit> ejectVoltage = new LoggedTunableMeasure<>("Intake/Voltages/Eject", Volts.of(-4));
    public static final LoggedTunableMeasure<VoltageUnit> holdVoltage = new LoggedTunableMeasure<>("Intake/Voltages/Hold", Volts.of(2));

    //Needs gamepiece pose & rotation
    public final GamepiecePose gamepiecePose = new GamepiecePose(
        new Transform3d(
            new Translation3d(

            ),
            new Rotation3d()
        )
    );
    //END OF NEEDS GAMEPIECE POSE

    public Intake(IntakeIO io){
        this.io = io;
        SmartDashboard.putData("Subsystems/Intake", this);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Intake", inputs);
    }

    //NEEDS REVIEW: Should motor direction be set in initialize?
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
    //END OF REVIEW NEEDED

    //NEEDS REVIEW: (are these commands we want?, do we need more?)
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
    // END OF REVIEW NEEDED
}