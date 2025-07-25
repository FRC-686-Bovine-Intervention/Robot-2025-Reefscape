package frc.robot.subsystems.vision;

import java.util.List;

import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.subsystems.vision.cameras.Camera;
import frc.robot.subsystems.vision.cameras.LimelightHelpers.LimelightResults;

public interface Pipeline {
    public LoggableInputs getInputs();
    public void setCamera(Camera camera);

    public void updateInputsFromPhotonResults(List<PhotonPipelineResult> results);
    public void updateInputsFromLimelightResults(LimelightResults results);

    public void clearInputs();
}
