package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.robotStructure.GamepiecePose;

public class Intake extends SubsystemBase{
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public final GamepiecePose gamepiecePose = new GamepiecePose(
        new Transform3d(
            new Translation3d(

            ),
            new Rotation3d()
        )
    );

    public Intake(IntakeIO io){
        this.io = io;
        SmartDashboard.putData("Subsystems/Intake", this);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Intake", inputs);
    }
}
