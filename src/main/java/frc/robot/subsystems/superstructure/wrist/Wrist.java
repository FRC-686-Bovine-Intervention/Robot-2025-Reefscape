package frc.robot.subsystems.superstructure.wrist;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.robotStructure.angle.ArmMech;

public class Wrist extends SubsystemBase {
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    public final ArmMech mech = new ArmMech(WristConstants.wristBase);

    public Wrist(WristIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Wrist", inputs);

        mech.set(inputs.encoder.position);
    }
    
    public void setVoltage(Measure<VoltageUnit> voltage) {
        io.setVoltage(voltage);
    }

    public void setAngle(Measure<AngleUnit> angle) {
        io.setAngle(angle);
    }

    public Command voltage(Supplier<Measure<VoltageUnit>> voltage) {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("Voltage");
            }

            @Override
            public void initialize() {
                execute();
            }
            @Override
            public void execute() {
                setVoltage(voltage.get());
            }
            @Override
            public void end(boolean interrupted) {
                io.stop();
            }
            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }

    public Command angle(Supplier<Measure<AngleUnit>> angle) {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("Angle");
            }

            @Override
            public void initialize() {
                execute();
            }
            @Override
            public void execute() {
                setAngle(angle.get());
            }
            @Override
            public void end (boolean interrupted) {
                io.stop();
            }
            @Override
            public boolean isFinished() {
                return false;
            }
        };
    }
}