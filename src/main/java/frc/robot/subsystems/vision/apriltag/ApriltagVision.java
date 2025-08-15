package frc.robot.subsystems.vision.apriltag;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.Arrays;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.AngleUnit;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.util.LoggedTracer;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;

public class ApriltagVision {
    private final ApriltagPipeline pipelines[];

    private static final LoggedTunableMeasure<AngleUnit> gyroTolerance = new LoggedTunableMeasure<>("Vision/Apriltags/Filtering/Gyro Tolerance", Degrees.of(10));
    private static final LoggedTunableNumber xyStdDevCoef = new LoggedTunableNumber("Vision/Apriltags/Std Devs/XY Coef", 0.4);
    private static final LoggedTunableNumber thetaStdDevCoef = new LoggedTunableNumber("Vision/Apriltags/Std Devs/Theta Coef", Double.POSITIVE_INFINITY);

    public ApriltagVision(ApriltagPipeline... pipelines) {
        System.out.println("[Init ApriltagVision] Instantiating ApriltagVision");
        this.pipelines = pipelines;
    }

    public void periodic() {
        for (var pipeline : this.pipelines) {
            var result = pipeline.getInputs();
            var loggingKey = "Vision/Apriltags/Results/" + pipeline.camera;
            var tracingKey = "VirtualSubsystem Periodic/ApriltagVision/Process Results/" + result.camMeta.hardwareName;
            var akitPose3d = new Pose3d[0];
            var akitTargetCorners = new Translation2d[0];
            for (var frame : result.frames) {
                var usableTags = Arrays
                    .stream(frame.targets)
                    .map((target) -> {
                        var optTagPose = FieldConstants.apriltagLayout.getTagPose(target.tagID);
                        if (optTagPose.isEmpty()) return Optional.empty();
                        return Optional.of(new AprilTag(target.tagID, optTagPose.get()));
                    })
                    .filter(Optional::isPresent)
                    .map(Optional::get)
                    .toArray(AprilTag[]::new)
                ;
                Logger.recordOutput(loggingKey + "/Targets/Tag IDs", Arrays.stream(usableTags).mapToInt((tag) -> tag.ID).toArray());
                Logger.recordOutput(loggingKey + "/Targets/Tag Poses", Arrays.stream(usableTags).map((tag) -> tag.pose).toArray(Pose3d[]::new));
                akitTargetCorners = Arrays.stream(frame.targets).flatMap((target) -> Arrays.stream(target.corners)).toArray(Translation2d[]::new);

                if (frame.targets.length == 0) continue;
    
                // final because averageTagDist mapToDouble needs it
                final Pose3d cameraPose3d;
                final Pose3d robotPose3d;
                var useVisionRotation = false;
    
                if (frame.targets.length >= 2) {
                    // TODO: multitag
                    // cameraPose3d = frame.estimatedCameraPose;
                    // robotPose3d = cameraPose3d.transformBy(pipeline.cameraConstants.mount.getRobotRelative().inverse());
                    // useVisionRotation = true;
                    cameraPose3d = null;
                    robotPose3d = null;
                    useVisionRotation = true;
                } else if (frame.targets.length == 1) {
                    var target = frame.targets[0];
                    var tagPose = FieldConstants.apriltagLayout.getTagPose(target.tagID).get();
                    var translationToTarget = target.bestCameraToTag.getTranslation();
                    var cameraRotation = pipeline.cameraConstants.mount.getFieldRelative().getRotation();
                    var tagRotationRelativeToCamera = tagPose.getRotation().minus(cameraRotation);
                    var cameraToTag = new Transform3d(translationToTarget, tagRotationRelativeToCamera);
                    var cameraPose = tagPose.transformBy(cameraToTag.inverse());
                    var robotPose = cameraPose.transformBy(pipeline.cameraConstants.mount.getRobotRelative().inverse());

                    cameraPose3d = cameraPose;
                    robotPose3d = robotPose;
                    useVisionRotation = false;
                    // var bestCameraPose = tagPose.transformBy(target.bestCameraToTag.inverse());
                    // var bestRobotPose = bestCameraPose.transformBy(result.camMeta.mount.getRobotRelative().inverse());
                    // var altCameraPose = tagPose.transformBy(target.altCameraToTag.inverse());
                    // var altRobotPose = altCameraPose.transformBy(result.camMeta.mount.getRobotRelative().inverse());
                    // if (frame.targets[0].poseAmbiguity < ambiguityThreshold.get()) {
                    //     var currentRotation = RobotState.getInstance().getPose().getRotation();
                    //     var bestRotation = bestRobotPose.getRotation().toRotation2d();
                    //     var altRotation = altRobotPose.getRotation().toRotation2d();
                    //     if (Math.abs(currentRotation.minus(bestRotation).getRadians()) < Math.abs(currentRotation.minus(altRotation).getRadians())) {
                    //         cameraPose3d = bestCameraPose;
                    //         robotPose3d = bestRobotPose;
                    //     } else {
                    //         cameraPose3d = altCameraPose;
                    //         robotPose3d = altRobotPose;
                    //     }
                    // } else {
                    //     cameraPose3d = null;
                    //     robotPose3d = null;
                    // }

                } else {
                    cameraPose3d = null;
                    robotPose3d = null;
                }
                if (robotPose3d == null || cameraPose3d == null) {
                    Logger.recordOutput(loggingKey + "/Robot pose null", robotPose3d == null);
                    Logger.recordOutput(loggingKey + "/Camera pose null", cameraPose3d == null);
                    continue;
                }
                Logger.recordOutput(loggingKey + "/Robot pose null", false);
                Logger.recordOutput(loggingKey + "/Camera pose null", false);
                var robotPose2d = robotPose3d.toPose2d();
                akitPose3d = new Pose3d[]{robotPose3d};
                Logger.recordOutput(loggingKey + "/Poses/Robot2d", robotPose2d);
                // Logger.recordOutput(loggingKey + "/Poses/Robot3d", robotPose3d);
                Logger.recordOutput(loggingKey + "/Poses/Camera3d", cameraPose3d);
    
                // Filtering
                var inField = ApriltagVisionConstants.acceptableFieldBox.withinBounds(robotPose2d.getTranslation());
                var closeToFloor = robotPose3d.getTranslation().getMeasureZ().isNear(Meters.zero(), ApriltagVisionConstants.zMargin);
                var closeToGyro = robotPose2d.getRotation().minus(RobotState.getInstance().getPose().getRotation()).getCos() > Math.cos(gyroTolerance.get().in(Radians));
                var gyroFilter = closeToGyro || usableTags.length >= 2;
    
                Logger.recordOutput(loggingKey + "/Filtering/In Field", inField);
                Logger.recordOutput(loggingKey + "/Filtering/Close to Floor", closeToFloor);
                Logger.recordOutput(loggingKey + "/Filtering/Close to Gyro", gyroFilter);
    
                if (
                    !inField
                    || !closeToFloor
                    || !gyroFilter
                ) {
                    continue;
                }
    
                // Std Devs
                var optAverageTagDistance = Arrays
                    .stream(usableTags)
                    .mapToDouble((tag) -> tag.pose.getTranslation().getDistance(cameraPose3d.getTranslation()))
                    .average()
                ;
                if (optAverageTagDistance.isEmpty()) continue;
                var averageTagDistance = optAverageTagDistance.getAsDouble();
                Logger.recordOutput(loggingKey + "/Std Devs/Average Distance", averageTagDistance);
    
                double xyStdDev =
                    xyStdDevCoef.get()
                    * averageTagDistance * averageTagDistance
                    / usableTags.length
                    * pipeline.cameraConstants.cameraStdCoef
                ;
                double thetaStdDev =
                    (useVisionRotation) ? (
                        thetaStdDevCoef.get()
                        * averageTagDistance * averageTagDistance
                        / usableTags.length
                        * pipeline.cameraConstants.cameraStdCoef
                    ) : (
                        Double.POSITIVE_INFINITY
                    )
                ;
                Logger.recordOutput(loggingKey + "/Std Devs/XY", xyStdDev);
                Logger.recordOutput(loggingKey + "/Std Devs/Theta", thetaStdDev);
    
                RobotState.getInstance().addVisionMeasurement(
                    robotPose2d,
                    VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev),
                    frame.timestamp
                );

                // robotPose = new AprilTagResultPose(robotPose2d, xyStdDev, thetaStdDev);
            }
            Logger.recordOutput(loggingKey + "/Poses/Robot3d", akitPose3d);
            Logger.recordOutput(loggingKey + "/Targets/Target Corners", akitTargetCorners);
            Logger.recordOutput(loggingKey + "/Frame Count", result.frames.length);
            LoggedTracer.logEpoch(tracingKey);
        }
        LoggedTracer.logEpoch("VirtualSubsystem Periodic/ApriltagVision/Process Results");
    }
}
