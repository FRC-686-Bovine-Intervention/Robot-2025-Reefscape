package frc.robot.subsystems.vision.apriltag;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConstants;
import frc.util.rust.iter.Iterator;

public class ApriltagCameraIOPhotonVision implements ApriltagCameraIO {
    private final PhotonCamera photonCam;
    private final CameraConstants camMeta;
    private PhotonPoseEstimator photonPoseEstimator;

    public ApriltagCameraIOPhotonVision(CameraConstants cam) {
        this.camMeta = cam;
        photonCam = new PhotonCamera(cam.hardwareName);

        photonPoseEstimator = new PhotonPoseEstimator(FieldConstants.apriltagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Transform3d.kZero);
        photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
    }

    @Override
    public void updateInputs(ApriltagCameraIOInputs inputs) {
        inputs.isConnected = photonCam.isConnected();

        if (!inputs.isConnected) return;
        var result = photonCam.getLatestResult();
        inputs.targets = result.getTargets().stream().map(ApriltagCameraIOPhotonVision::targetFromPhotonTarget).toArray(ApriltagCameraTarget[]::new);

        if (photonPoseEstimator == null) return;
        photonPoseEstimator.setRobotToCameraTransform(camMeta.mount.getRobotRelative());
        photonPoseEstimator.setReferencePose(RobotState.getInstance().getPose());

        var optRobotPose = photonPoseEstimator.update(result);
        optRobotPose.ifPresent((e) -> {
            inputs.timestamp = e.timestampSeconds;
            inputs.estimatedRobotPose = e.estimatedPose;
        });
    }

    private static ApriltagCameraTarget targetFromPhotonTarget(PhotonTrackedTarget photonTarget) {
        return new ApriltagCameraTarget(
            photonTarget.getFiducialId(),
            photonTarget.getBestCameraToTarget(),
            photonTarget.getAlternateCameraToTarget(),
            photonTarget.getPoseAmbiguity(),
            Iterator.of(photonTarget.getDetectedCorners()).map((corner) -> new Translation2d(corner.x, corner.y)).collect_array(Translation2d[]::new)
        );
    }
}
