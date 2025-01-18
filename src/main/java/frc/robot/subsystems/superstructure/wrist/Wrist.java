package frc.robot.subsystems.superstructure.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase{

    private WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Wrist", inputs);
    }
    
    public void setVoltage (Measure<VoltageUnit> voltage) {
        io.setVoltage(voltage);
    }

    public void setRotation (Measure<AngleUnit> angle) {
        io.setAngle(angle);
    }

    public Command voltage (Measure<VoltageUnit> voltage) {
        var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("setVoltageTo");
            }

            @Override
            public void initialize() {
                // TODO Auto-generated method stub
                super.initialize();
            }
            @Override
            public void execute() {
                // TODO Auto-generated method stub
                super.execute();
            }
            @Override
            public void end(boolean interrupted) {
                // TODO Auto-generated method stub
                super.end(interrupted);
            }
            @Override
            public boolean isFinished() {
                // TODO Auto-generated method stub
                return super.isFinished();
            }
        };
    }

}
