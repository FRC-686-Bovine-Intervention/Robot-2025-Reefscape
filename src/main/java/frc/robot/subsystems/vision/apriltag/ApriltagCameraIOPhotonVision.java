package frc.robot.subsystems.vision.apriltag;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConstants;
import frc.util.rust.Option;
import frc.util.rust.iter.Iterator;

public class ApriltagCameraIOPhotonVision implements ApriltagCameraIO {
    private final PhotonCamera photonCam;
    private final CameraConstants camMeta;
    private final PhotonPoseEstimator photonPoseEstimator;

    public ApriltagCameraIOPhotonVision(CameraConstants cam) {
        this.camMeta = cam;
        photonCam = new PhotonCamera(cam.hardwareName);

        photonPoseEstimator = new PhotonPoseEstimator(FieldConstants.apriltagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Transform3d.kZero);
        photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void updateInputs(ApriltagCameraIOInputs inputs) {
        inputs.isConnected = photonCam.isConnected();

        if (!inputs.isConnected) return;
        var results = photonCam.getAllUnreadResults();

        inputs.frames = Iterator.of(results).map(this::frameFromResult).collect_array(ApriltagCameraFrame[]::new);
    }

    private ApriltagCameraFrame frameFromResult(PhotonPipelineResult result) {
        var timestamp = result.getTimestampSeconds();
        var estimatedPose = photonPoseEstimator.update(result);
        var targets = Iterator.of(result.getTargets()).map(this::targetFromPhotonTarget).collect_array(ApriltagCameraTarget[]::new);
        return new ApriltagCameraFrame(
            timestamp,
            Option.from(estimatedPose).map((p) -> p.estimatedPose).unwrap_to_nullable(),
            targets
        );
    }

    private ApriltagCameraTarget targetFromPhotonTarget(PhotonTrackedTarget photonTarget) {
        return new ApriltagCameraTarget(
            photonTarget.getFiducialId(),
            photonTarget.getBestCameraToTarget(),
            photonTarget.getAlternateCameraToTarget(),
            photonTarget.getPoseAmbiguity(),
            Iterator.of(photonTarget.getDetectedCorners()).map((corner) -> new Translation2d(corner.x, corner.y)).collect_array(Translation2d[]::new)
        );
    }
}
