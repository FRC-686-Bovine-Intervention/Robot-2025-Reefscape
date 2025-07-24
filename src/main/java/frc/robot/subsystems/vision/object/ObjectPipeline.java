package frc.robot.subsystems.vision.object;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.List;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.vision.Pipeline;
import frc.robot.subsystems.vision.cameras.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.vision.object.ObjectPipeline.ObjectPipelineInputs.ObjectFrame;
import frc.robot.subsystems.vision.object.ObjectPipeline.ObjectPipelineInputs.ObjectTarget;
import frc.util.loggerUtil.LoggerUtil;

public class ObjectPipeline implements Pipeline {
    public final ObjectPipelineInputs inputs = new ObjectPipelineInputs();

    @Override
    public LoggableInputs getInputs() {
        return this.inputs;
    }

    @Override
    public void updateFromPhotonResults(List<PhotonPipelineResult> results) {
        this.inputs.frames = results.stream()
            .map((result) -> {
                var timestamp = result.getTimestampSeconds();
                var targets = result.getTargets().stream()
                    .map((target) -> {
                        var cameraToTargetVector = new Translation3d(1, 0, 0)
                            .rotateBy(new Rotation3d(0, 0, Radians.convertFrom(-target.getYaw(), Degrees)))
                            .rotateBy(new Rotation3d(0, Radians.convertFrom(-target.getPitch(), Degrees), 0))
                        ;
                        var area = target.getArea();
                        var corners = target.getDetectedCorners().stream().map((corner) -> new Translation2d(corner.x, corner.y)).toArray(Translation2d[]::new);
                        return new ObjectTarget(cameraToTargetVector, area, corners);
                    })
                    .toArray(ObjectTarget[]::new)
                ;
                return new ObjectFrame(timestamp, targets);
            })
            .toArray(ObjectFrame[]::new)
        ;
    }

    @Override
    public void updateFromLimelightResults(LimelightResults results) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateFromLimelightResults'");
    }

    @Override
    public void clear() {
        this.inputs.frames = new ObjectFrame[0];
    }

    public static class ObjectPipelineInputs implements LoggableInputs {
        public ObjectFrame[] frames = new ObjectFrame[0];

        @Override
        public void toLog(LogTable table) {
            LoggerUtil.toLogArray(table.getSubtable("Frames"), this.frames);
        }

        @Override
        public void fromLog(LogTable table) {
            LoggerUtil.fromLogArray(table.getSubtable("Frames"), ObjectFrame::new, ObjectFrame[]::new);
        }

        public static class ObjectFrame implements LoggableInputs {
            public double timestamp;
            public ObjectTarget[] targets;

            public ObjectFrame() {}

            public ObjectFrame(double timestamp, ObjectTarget[] targets) {
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
                this.targets = LoggerUtil.fromLogArray(table.getSubtable("Targets"), ObjectTarget::new, ObjectTarget[]::new);
            }
        }

        public static class ObjectTarget implements LoggableInputs {
            public Translation3d cameraToTargetVector;
            public double area;
            public Translation2d[] corners;

            public ObjectTarget() {}

            public ObjectTarget(Translation3d cameraToTargetVector, double area, Translation2d[] corners) {
                this.cameraToTargetVector = cameraToTargetVector;
                this.area = area;
                this.corners = corners;
            }

            @Override
            public void toLog(LogTable table) {
                table.put("CameraToTargetVector", this.cameraToTargetVector);
                table.put("Area", this.area);
                table.put("Corners", this.corners);
            }
            @Override
            public void fromLog(LogTable table) {
                this.cameraToTargetVector = table.get("CameraToTargetVector", this.cameraToTargetVector);
                this.area = table.get("Area", this.area);
                this.corners = table.get("Corners", this.corners);
            }
        }
    }
}
