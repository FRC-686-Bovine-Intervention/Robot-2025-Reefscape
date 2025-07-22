package frc.robot;

import org.ejml.equation.IntegerSequence.For;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import frc.util.rust.iter.IntoIterator;

public class RobotState {
    private static RobotState instance;
    public static RobotState getInstance() {if (instance == null) {instance = new RobotState();} return instance;}

    private SwerveDrivePoseEstimator poseEstimator;
    private Matrix<N3, N1> robotPoseStdDevs = VecBuilder.fill(0,0,0);
    
    private Pose2d[] reefObjectivePoses = new Pose2d[4];

    public void initializePoseEstimator(
        SwerveDriveKinematics kinematics,
        Rotation2d gyroAngle,
        SwerveModulePosition[] modulePositions,
        Pose2d initialPoseMeters
    ) {
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions, initialPoseMeters);
    }

    public void addDriveMeasurement(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        poseEstimator.update(rotation, modulePositions);
    }

    public void addVisionMeasurement(Pose2d pose, Matrix<N3, N1> stdDevs, double timestamp) {
        poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    public void log() {
        Logger.recordOutput("Odometry/Robot", getPose());
        // Logger.recordOutput("Odometry/Std Devs", robotPoseStdDevs);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getReefPose() {
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
    }

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
}
