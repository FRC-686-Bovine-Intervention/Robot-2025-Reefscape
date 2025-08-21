package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class RobotState {
    private static RobotState instance;
    public static RobotState getInstance() {if (instance == null) {instance = new RobotState();} return instance;}

    private SwerveDrivePoseEstimator poseEstimator;

    public void initializePoseEstimator(
        SwerveDriveKinematics kinematics,
        Rotation2d gyroAngle,
        SwerveModulePosition[] modulePositions,
        Pose2d initialPoseMeters
    ) {
        this.poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions, initialPoseMeters);
    }

    public void addOdometryObservation(OdometryObservation observation) {
        this.poseEstimator.updateWithTime(observation.timestamp(), observation.gyroRotation.toRotation2d(), observation.modulePositions());
    }

    public void addVisionMeasurement(Pose2d pose, Matrix<N3, N1> stdDevs, double timestamp) {
        this.poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    public void log() {
        Logger.recordOutput("Odometry/Robot", this.getPose());
    }

    public Pose2d getPose() {
        return this.poseEstimator.getEstimatedPosition();
    }

    public void setPose(
        Rotation2d rotation,
        SwerveModulePosition[] modulePositions,
        Pose2d fieldToVehicle
    ) {
        this.setPose(rotation, modulePositions, fieldToVehicle, VecBuilder.fill(0,0,0));
    }
    public void setPose(
        Rotation2d rotation,
        SwerveModulePosition[] modulePositions,
        Pose2d fieldToVehicle,
        Matrix<N3, N1> stdDevs
    ) {
        this.poseEstimator.resetPosition(rotation, modulePositions, fieldToVehicle);
    }

    public static record OdometryObservation(
        double timestamp,
        Rotation3d gyroRotation,
        SwerveModulePosition[] modulePositions
    ) {}
}
