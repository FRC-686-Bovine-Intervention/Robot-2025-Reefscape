package frc.robot.subsystems.vision.apriltag;

import frc.robot.subsystems.vision.cameras.Camera;
import frc.robot.subsystems.vision.cameras.CameraIO.CameraFrame;

public class ApriltagPipeline {
    public final Camera camera;
    public final int pipelineIndex;
    public final double pipelineStdScale;

    public ApriltagPipeline(Camera camera, int pipelineIndex, double pipelineStdScale) {
        this.camera = camera;
        this.pipelineIndex = pipelineIndex;
        this.pipelineStdScale = pipelineStdScale;
    }

    public CameraFrame[] getFrames() {
        return this.camera.getPipelineFrames(this.pipelineIndex);
    }
}
