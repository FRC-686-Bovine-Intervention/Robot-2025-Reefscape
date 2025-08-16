package frc.robot.subsystems.vision.object;

import frc.robot.subsystems.vision.cameras.Camera;
import frc.robot.subsystems.vision.cameras.CameraIO.CameraFrame;

public class ObjectPipeline {
    public final Camera camera;
    public final int pipelineIndex;

    public ObjectPipeline(Camera camera, int pipelineIndex) {
        this.camera = camera;
        this.pipelineIndex = pipelineIndex;
    }

    public CameraFrame[] getFrames() {
        return this.camera.getPipelineFrames(this.pipelineIndex);
    }
}
