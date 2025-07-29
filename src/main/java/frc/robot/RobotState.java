package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef.RackObject;
import frc.robot.subsystems.vision.apriltag.ApriltagVisionConstants;
import frc.util.loggerUtil.tunables.LoggedTunableMeasure;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;

public class RobotState {
    private static RobotState instance;
    public static RobotState getInstance() {if (instance == null) {instance = new RobotState();} return instance;}

    private static final LoggedTunableNumber txTyObservationStaleSecs = new LoggedTunableNumber("RobotState/TxTyObservationStaleSeconds", 0.2);
    private static final LoggedTunableMeasure<DistanceUnit> minDistanceTagPoseBlend = new LoggedTunableMeasure<>("RobotState/MinDistanceTagPoseBlend", Inches.of(24.0));
    private static final LoggedTunableMeasure<DistanceUnit> maxDistanceTagPoseBlend = new LoggedTunableMeasure<>("RobotState/MaxDistanceTagPoseBlend", Inches.of(36));

    private SwerveDrivePoseEstimator poseEstimator;
    private SwerveDrivePoseEstimator odometryPoseEstimator;
    private Matrix<N3, N1> robotPoseStdDevs = VecBuilder.fill(0,0,0);
    
    //private Pose2d[] reefObjectivePoses = new Pose2d[4];

    private static final double poseBufferSizeSec = 2.0;
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

    private final Map<Integer, TxTyPoseRecord> txTyPoses = new HashMap<>();

    public void initializePoseEstimator(
        SwerveDriveKinematics kinematics,
        Rotation2d gyroAngle,
        SwerveModulePosition[] modulePositions,
        Pose2d initialPoseMeters
    ) {
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions, initialPoseMeters);
        odometryPoseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions, initialPoseMeters);
    }

    public void addDriveMeasurement(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        poseEstimator.update(rotation, modulePositions);
        odometryPoseEstimator.update(rotation, modulePositions);
        poseBuffer.addSample(Timer.getTimestamp(), getPose());
    }

    public void addVisionMeasurement(Pose2d pose, Matrix<N3, N1> stdDevs, double timestamp) {
        poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
        poseBuffer.addSample(Timer.getTimestamp(), getPose());
    }

    public void addTxTyObservation(TxTyObservation observation) {
        if (txTyPoses.containsKey(observation.tagId())
            && txTyPoses.get(observation.tagId()).timestamp >= observation.timestamp()){
            return;
        }

        var sample = poseBuffer.getSample(observation.timestamp());
        if (sample.isEmpty()) {
            return;
        }

        Rotation2d robotRotation = getPose().transformBy(new Transform2d(getOdometryOnlyPose(), sample.get())).getRotation();

        var camMeta = ApriltagVisionConstants.findApriltagCameraConstantsByID(observation.camera);
        var tagPose = FieldConstants.apriltagLayout.getTagPose(observation.tagId).get();
        var cameraMountAngleY = camMeta.mount.getRobotRelative().getRotation().getMeasureY();
        var cameraMountAngleZ = camMeta.mount.getRobotRelative().getRotation().toRotation2d().rotateBy(robotRotation).getMeasure();
        var tagTYtoRobot = cameraMountAngleY.plus(observation.ty);
        var cameraDistanceHorizontalToTarget = Math.cos(tagTYtoRobot.in(Radians)) * observation.distance.in(Meters);
        var cameraAngleToTarget = tagPose.getRotation().getMeasureZ().minus(Degrees.of(180)).minus(cameraMountAngleZ);
        var oppositeAngle = cameraAngleToTarget.plus(observation.tx);
        var cameraToTag = new Transform3d(
            new Translation3d(
                Meters.of(cameraDistanceHorizontalToTarget * Math.cos(oppositeAngle.in(Radians))),
                Meters.of(-1 * cameraDistanceHorizontalToTarget * Math.sin(oppositeAngle.in(Radians))),
                camMeta.mount.getRobotRelative().getTranslation().getMeasureZ()
            ),
            new Rotation3d(
                camMeta.mount.getRobotRelative().getRotation().getMeasureX(),
                cameraMountAngleY,
                cameraMountAngleZ
            )
        );
        var cameraPose = tagPose.transformBy(cameraToTag.inverse());
        var robotPose = cameraPose.transformBy(camMeta.mount.getRobotRelative().inverse());
        var robotPose2d = robotPose.toPose2d();
        txTyPoses.put(
            observation.tagId(),
            new TxTyPoseRecord(robotPose2d, observation.distance(), observation.timestamp())
        );
    }

    public void log() {
        Logger.recordOutput("Odometry/Robot", getPose());
        // Logger.recordOutput("Odometry/Std Devs", robotPoseStdDevs);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getOdometryOnlyPose() {
        return odometryPoseEstimator.getEstimatedPosition();
    }

    public Optional<Pose2d> getTxTyPose(int tagID) {
        if (!txTyPoses.containsKey(tagID)) {
            return Optional.empty();
        }
        var data = txTyPoses.get(tagID);

        if (Timer.getTimestamp() - data.timestamp() >= txTyObservationStaleSecs.get()) {
            return Optional.empty();
        }

        var sample = poseBuffer.getSample(data.timestamp());

        return sample.map(pose2d -> data.pose().plus(new Transform2d(pose2d, getOdometryOnlyPose())));
    }


    public ReefPoseEstimate getReefPose(RackObject rack, Pose2d finalPose) {
        var tagPose = getTxTyPose(rack.apriltagID);

        if (tagPose.isEmpty()) return new ReefPoseEstimate(getPose(), 0.0);

        final double t = 
            MathUtil.clamp(
                (getPose().getTranslation().getDistance(finalPose.getTranslation()) - minDistanceTagPoseBlend.get().in(Meters)) / (maxDistanceTagPoseBlend.get().minus(minDistanceTagPoseBlend.get()).in(Meters)),
                0.0,
                1.0);

        return new ReefPoseEstimate(getPose().interpolate(tagPose.get(), 1.0 - t), 1.0 - t);
    }
    /*public Pose2d getReefPose() {
        double totalX = 0;
        double totalY = 0;
        double sumSin = 0;
        double sumCos = 0;
        int total_valid = 0;
        
        for (Pose2d reefObjectivePose : reefObjectivePoses){
            if (reefObjectivePose != null) {
                total_valid += 1;

                totalX += reefObjectivePose.getX();
                totalY += reefObjectivePose.getY();

                double angle = reefObjectivePose.getRotation().getRadians();
                sumSin += Math.sin(angle);
                sumCos += Math.cos(angle);
            }
        }
        if(total_valid != 0){
            double avgX = totalX / total_valid;
            double avgY = totalY / total_valid;
            double avgAngle = Math.atan2(sumSin / total_valid, sumCos / total_valid);
            return new Pose2d(avgX, avgY, new Rotation2d(avgAngle));
        } else {
            return null;
        }
    }

    public void updateReefPose(int cam_id, Pose2d pose) {
        reefObjectivePoses[cam_id] = pose;
    }*/

    public void setPose(
        Rotation2d rotation,
        SwerveModulePosition[] modulePositions,
        Pose2d fieldToVehicle
    ) {
        setPose(rotation, modulePositions, fieldToVehicle, VecBuilder.fill(0,0,0));
    }
    public void setPose(
        Rotation2d rotation,
        SwerveModulePosition[] modulePositions,
        Pose2d fieldToVehicle,
        Matrix<N3, N1> stdDevs
    ) {
        poseEstimator.resetPosition(rotation, modulePositions, fieldToVehicle);
        robotPoseStdDevs = stdDevs;
    }

    public record TxTyObservation(int tagId, int camera, Angle tx, Angle ty, Distance distance, double timestamp) {};

    public record TxTyPoseRecord(Pose2d pose, Distance distance, double timestamp) {}

    public record ReefPoseEstimate(Pose2d pose, double blend) {}
}
