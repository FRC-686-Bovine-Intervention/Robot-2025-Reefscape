package frc.robot.subsystems.vision.cameras;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.vision.Pipeline;
import frc.robot.subsystems.vision.cameras.CameraIO.CameraIOInputs;

public class Camera implements Subsystem {
    private final CameraIO io;
    private final CameraIOInputs inputs;
    private final Pipeline[] pipelines;

    public Camera(CameraIO io, Pipeline... pipelines) {
        this.io = io;
        this.pipelines = pipelines;
        this.inputs = new CameraIOInputs(Arrays.stream(this.pipelines).map(Pipeline::getInputs).toArray(LoggableInputs[]::new));
        this.register();
    }

    public void processInputs() {
        this.io.updateInputs(this.inputs, this.pipelines);
        Logger.processInputs(null, this.inputs);
    }
}
