package frc.robot.subsystems.vision.apriltag;

import java.util.List;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.Pipeline;
import frc.robot.subsystems.vision.apriltag.ApriltagPipeline.ApriltagPipelineInputs.ApriltagFrame;
import frc.robot.subsystems.vision.apriltag.ApriltagPipeline.ApriltagPipelineInputs.ApriltagTarget;
import frc.robot.subsystems.vision.apriltag.ApriltagVisionConstants.ApriltagCameraConstants;
import frc.robot.subsystems.vision.cameras.LimelightHelpers.LimelightResults;
import frc.util.loggerUtil.LoggerUtil;

public class ApriltagPipeline implements Pipeline {
    private final ApriltagPipelineInputs inputs = new ApriltagPipelineInputs();
    public final ApriltagCameraConstants cameraConstants;

    public ApriltagPipeline(ApriltagCameraConstants cameraConstants) {
        this.cameraConstants = cameraConstants;
    }

    @Override
    public ApriltagPipelineInputs getInputs() {
        return this.inputs;
    }

    @Override
    public void updateFromPhotonResults(List<PhotonPipelineResult> results) {
        this.inputs.frames = results.stream()
            .map((result) -> {
                var timestamp = result.getTimestampSeconds();
                var targets = result.getTargets().stream()
                    .map((target) -> {
                        var tagID = target.getFiducialId();
                        var bestCameraToTag = target.getBestCameraToTarget();
                        var altCameraToTag = target.getAlternateCameraToTarget();
                        var poseAmbiguity = target.getPoseAmbiguity();
                        var corners = target.getDetectedCorners().stream().map((corner) -> new Translation2d(corner.x, corner.y)).toArray(Translation2d[]::new);
                        return new ApriltagTarget(tagID, bestCameraToTag, altCameraToTag, poseAmbiguity, corners);
                    })
                    .toArray(ApriltagTarget[]::new)
                ;
                return new ApriltagFrame(timestamp, targets);
            })
            .toArray(ApriltagFrame[]::new)
        ;
    }

    @Override
    public void updateFromLimelightResults(LimelightResults results) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateFromLimelightResults'");
    }

    @Override
    public void clear() {
        this.inputs.frames = new ApriltagFrame[0];
    }

    public static class ApriltagPipelineInputs implements LoggableInputs {
        public ApriltagFrame[] frames = new ApriltagFrame[0];

        @Override
        public void toLog(LogTable table) {
            LoggerUtil.toLogArray(table.getSubtable("Frames"), this.frames);
        }

        @Override
        public void fromLog(LogTable table) {
            LoggerUtil.fromLogArray(table.getSubtable("Frames"), ApriltagFrame::new, ApriltagFrame[]::new);
        }

        public static class ApriltagFrame implements LoggableInputs {
            public double timestamp;
            // TODO: multitag result
            public ApriltagTarget[] targets;

            public ApriltagFrame() {}

            public ApriltagFrame(double timestamp, ApriltagTarget[] targets) {
                this.timestamp = timestamp;
                this.targets = targets;
            }

            @Override
            public void toLog(LogTable table) {
                table.put("Timestamp", this.timestamp);
                LoggerUtil.toLogArray(table.getSubtable("Targets"), this.targets);
            }

            @Override
            public void fromLog(LogTable table) {
                this.timestamp = table.get("Timestamp", this.timestamp);
                this.targets = LoggerUtil.fromLogArray(table.getSubtable("Targets"), ApriltagTarget::new, ApriltagTarget[]::new);
            }
        }

        public static class ApriltagTarget implements LoggableInputs {
            public int tagID;
            public Transform3d bestCameraToTag;
            public Transform3d altCameraToTag;
            public double poseAmbiguity;
            public Translation2d[] corners;

            public ApriltagTarget() {}

            public ApriltagTarget(int tagID, Transform3d bestCameraToTag, Transform3d altCameraToTag, double poseAmbiguity, Translation2d[] corners) {
                this.tagID = tagID;
                this.bestCameraToTag = bestCameraToTag;
                this.altCameraToTag = altCameraToTag;
                this.poseAmbiguity = poseAmbiguity;
                this.corners = corners;
            }

            @Override
            public void toLog(LogTable table) {
                table.put("TagID", this.tagID);
                table.put("BestCameraToTag", this.bestCameraToTag);
                table.put("AltCameraToTag", this.altCameraToTag);
                table.put("PoseAmbiguity", this.poseAmbiguity);
                table.put("Corners", this.corners);
            }
            @Override
            public void fromLog(LogTable table) {
                this.tagID = table.get("TagID", this.tagID);
                this.bestCameraToTag = table.get("BestCameraToTag", this.bestCameraToTag);
                this.altCameraToTag = table.get("AltCameraToTag", this.altCameraToTag);
                this.poseAmbiguity = table.get("PoseAmbiguity", this.poseAmbiguity);
                this.corners = table.get("Corners", this.corners);
            }
        }
    }
}
