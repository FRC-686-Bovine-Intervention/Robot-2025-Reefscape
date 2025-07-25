package frc.robot.subsystems.vision.cameras;

import java.util.Arrays;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.vision.Pipeline;
import frc.robot.subsystems.vision.cameras.CameraIO.CameraIOInputs;
import frc.util.led.animation.StatusLightAnimation;
import frc.util.robotStructure.CameraMount;

public class Camera implements Subsystem {
    private final CameraIO io;
    private final CameraIOInputs inputs;
    private final Pipeline[] pipelines;
    private final String name;
    private final Alert disconnectedAlert;
    private final Optional<StatusLightAnimation> connectionAnimation;
    public final CameraMount mount;

    public Camera(CameraIO io, String name, Transform3d cameraBase, Optional<StatusLightAnimation> connectionAnimation, Pipeline... pipelines) {
        this.io = io;
        this.pipelines = pipelines;
        this.inputs = new CameraIOInputs(Arrays.stream(this.pipelines).map(Pipeline::getInputs).toArray(LoggableInputs[]::new));
        this.name = name;
        this.mount = new CameraMount(cameraBase);
        this.disconnectedAlert = new Alert("Camera \"" + this.name + "\" is not connected", AlertType.kError);
        this.connectionAnimation = connectionAnimation;
        Arrays.stream(this.pipelines).forEach((pipeline) -> pipeline.setCamera(this));
        
        this.register();
    }

    public void processInputs() {
        this.io.updateInputs(this.inputs, this.pipelines);
        Logger.processInputs("Inputs/Camera/" + this.name, this.inputs);

        this.disconnectedAlert.set(!this.inputs.isConnected);
        this.connectionAnimation.ifPresent((animation) -> animation.setStatus(this.inputs.isConnected));
    }
}
