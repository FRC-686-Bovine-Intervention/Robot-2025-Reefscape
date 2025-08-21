package frc.robot.subsystems.vision.cameras;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Translation2d;

public class CameraIOPhoton implements CameraIO {
    private final PhotonCamera photonCam;

    public CameraIOPhoton(String name) {
        this.photonCam = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        inputs.isConnected = this.photonCam.isConnected();
        var selectedPipeline = this.photonCam.getPipelineIndex();
        inputs.frames = this.photonCam.getAllUnreadResults().stream()
            .map((result) -> {
                var timestamp = result.getTimestampSeconds();
                var targets = result.getTargets().stream()
                    .map((target) -> {
                        var tagID = target.getFiducialId();
                        var yawRads = target.getYaw();
                        var pitchRads = target.getPitch();
                        var skewRads = target.getSkew();
                        var bestCameraToTag = target.getBestCameraToTarget();
                        var altCameraToTag = target.getAlternateCameraToTarget();
                        var poseAmbiguity = target.getPoseAmbiguity();
                        var corners = target.getDetectedCorners().stream().map((corner) -> new Translation2d(corner.x, corner.y)).toArray(Translation2d[]::new);
                        return new CameraTarget(
                            tagID,
                            yawRads,
                            pitchRads,
                            skewRads,
                            bestCameraToTag,
                            altCameraToTag,
                            poseAmbiguity,
                            corners
                        );
                    })
                    .toArray(CameraTarget[]::new)
                ;
                var multiTagResult = result.multitagResult
                    .map((multi) -> new MultiTagResult(
                        multi.fiducialIDsUsed.stream().mapToInt(Short::intValue).toArray(),
                        multi.estimatedPose.best,
                        multi.estimatedPose.altReprojErr,
                        multi.estimatedPose.alt,
                        multi.estimatedPose.altReprojErr,
                        multi.estimatedPose.ambiguity
                    ))
                ;
                return new CameraFrame(timestamp, selectedPipeline, targets, multiTagResult);
            })
            .toArray(CameraFrame[]::new)
        ;
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
