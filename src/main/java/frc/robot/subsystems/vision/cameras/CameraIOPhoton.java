package frc.robot.subsystems.vision.cameras;

import org.photonvision.PhotonCamera;

import frc.robot.subsystems.vision.Pipeline;

public class CameraIOPhoton implements CameraIO {
    private final PhotonCamera photonCam;

    public CameraIOPhoton(String name) {
        this.photonCam = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs, Pipeline[] pipelines) {
        inputs.isConnected = this.photonCam.isConnected();
        var selectedPipeline = this.photonCam.getPipelineIndex();
        for (int i = 0; i < pipelines.length; i++) {
            if (i == selectedPipeline) {
                pipelines[i].updateInputsFromPhotonResults(this.photonCam.getAllUnreadResults());
            } else {
                pipelines[i].clearInputs();
            }
        }
    }

    @Override
    public void setPipeline(int pipelineIndex) {
        this.photonCam.setPipelineIndex(pipelineIndex);
    }

    @Override
    public void setDriverMode(boolean driverMode) {
        this.photonCam.setDriverMode(driverMode);
    }
}
