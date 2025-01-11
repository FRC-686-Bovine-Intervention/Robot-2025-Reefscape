package frc.robot.subsystems.vision.apriltag;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public interface ApriltagCameraIO {

    public class ApriltagCameraIOInputs implements LoggableInputs {
        public boolean isConnected;
        public double timestamp;
        public ApriltagCameraTarget[] targets = new ApriltagCameraTarget[0];
        public Pose3d estimatedRobotPose = new Pose3d();

        @Override
        public void toLog(LogTable table) {
            table.put("IsConnected", isConnected);
            table.put("Timestamp", timestamp);
            table.put("EstimatedRobotPose", estimatedRobotPose);
            var targetsTable = table.getSubtable("Targets");
            targetsTable.put("length", targets.length);
            for (int i = 0; i < targets.length ; i++) {
                var target = targets[i];
                var targetTable = targetsTable.getSubtable(Integer.toString(i));
                targetTable.put("TagID", target.tagID);
                targetTable.put("BestCameraToTag", target.bestCameraToTag);
                targetTable.put("AltCameraToTag", target.altCameraToTag);
                targetTable.put("PoseAmbiguity", target.poseAmbiguity);
                targetTable.put("Corners", target.corners);
            }
        }
        @Override
        public void fromLog(LogTable table) {
            this.isConnected = table.get("IsConnected", isConnected);
            this.timestamp = table.get("Timestamp", timestamp);
            this.estimatedRobotPose = table.get("EstimatedRobotPose", estimatedRobotPose);
            var targetsTable = table.getSubtable("Targets");
            this.targets = new ApriltagCameraTarget[targetsTable.get("length", 0)];
            for (int i = 0; i < targets.length ; i++) {
                var targetTable = targetsTable.getSubtable(Integer.toString(i));
                targets[i] = new ApriltagCameraTarget(
                    targetTable.get("TagID", -1),
                    targetTable.get("BestCameraToTag", Transform3d.kZero),
                    targetTable.get("AltCameraToTag", Transform3d.kZero),
                    targetTable.get("PoseAmbiguity", -1),
                    targetTable.get("Corners", new Translation2d[0])
                );
            }
        }
    }

    public default void updateInputs(ApriltagCameraIOInputs inputs) {}

    public static class ApriltagCameraTarget {
        public final int tagID;
        public final Transform3d bestCameraToTag;
        public final Transform3d altCameraToTag;
        public final double poseAmbiguity;
        public final Translation2d[] corners;

        public ApriltagCameraTarget(int tagID, Transform3d bestCameraToTag, Transform3d altCameraToTag, double poseAmbiguity, Translation2d[] corners) {
            this.tagID = tagID;
            this.bestCameraToTag = bestCameraToTag;
            this.altCameraToTag = altCameraToTag;
            this.poseAmbiguity = poseAmbiguity;
            this.corners = corners;
        }

        // public static final ApriltagCameraTargetStruct struct = new ApriltagCameraTargetStruct();
        // public static class ApriltagCameraTargetStruct implements Struct<ApriltagCameraTarget> {
        //     @Override
        //     public Class<ApriltagCameraTarget> getTypeClass() {
        //         return ApriltagCameraTarget.class;
        //     }

        //     @Override
        //     public String getTypeName() {
        //         return "ApriltagTarget";
        //     }

        //     @Override
        //     public int getSize() {
        //         return kSizeInt32 * 1 + Transform3d.struct.getSize() * 2 + kSizeDouble * 1;
        //     }

        //     @Override
        //     public String getSchema() {
        //         return "int tagID;Transform3d bestCameraToTag;Transform3d altCameraToTag";
        //     }

        //     @Override
        //     public Struct<?>[] getNested() {
        //         return new Struct[]{Transform3d.struct,Transform3d.struct};
        //     }

        //     @Override
        //     public ApriltagCameraTarget unpack(ByteBuffer bb) {
        //         var tagID = bb.getInt();
        //         var bestCameraToTag = Transform3d.struct.unpack(bb);
        //         var altCameraToTag = Transform3d.struct.unpack(bb);
        //         var poseAmbiguity = bb.getDouble();
        //         return new ApriltagCameraTarget(tagID, bestCameraToTag, altCameraToTag, poseAmbiguity);
        //     }

        //     @Override
        //     public void pack(ByteBuffer bb, ApriltagCameraTarget value) {
        //         bb.putInt(value.tagID);
        //         Transform3d.struct.pack(bb, value.bestCameraToTag);
        //         Transform3d.struct.pack(bb, value.altCameraToTag);
        //         bb.putDouble(value.poseAmbiguity);
        //     }
        // }
    }
}
