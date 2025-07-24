package frc.robot.subsystems.vision;

import java.util.List;

import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.subsystems.vision.cameras.LimelightHelpers.LimelightResults;

public interface Pipeline {
    public LoggableInputs getInputs();

    public void updateFromPhotonResults(List<PhotonPipelineResult> results);
    public void updateFromLimelightResults(LimelightResults results);

    public void clear();
}
