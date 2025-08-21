package frc.robot;

import java.util.NoSuchElementException;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drive.DriveConstants;

public class RobotState {
    private static RobotState instance;
    public static RobotState getInstance() {if (instance == null) {instance = new RobotState();} return instance;}

    private Pose2d odometryPose = Pose2d.kZero;
    private Rotation3d gyroOffset = Rotation3d.kZero;
    private static final Matrix<N3, N1> odometryStateStdDevs = VecBuilder.fill(0.003, 0.003, 0.002);
    private final Matrix<N3, N1> qStdDevs;
    
    private static final double poseBufferSizeSecs = 2.0;
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSecs);
    private Pose2d estimatedGlobalPose = Pose2d.kZero;

    private RobotState() {
        this.qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
        for (int i = 0; i < 3; i++) {
            this.qStdDevs.set(i, 0, Math.pow(odometryStateStdDevs.get(i, 0), 2));
        }
    }

    public void log() {
        Logger.recordOutput("RobotState/OdometryPose", this.odometryPose);
        Logger.recordOutput("RobotState/EstimatedGlobalPose", this.getEstimatedGlobalPose());
    }

    public Pose2d getEstimatedGlobalPose() {
        return this.estimatedGlobalPose;
    }

    public void resetPose(Pose2d pose) {
        var gyroOffsetYaw = this.gyroOffset.toRotation2d();
        var gyroOffsetNoYaw = this.gyroOffset.minus(new Rotation3d(gyroOffsetYaw));

        this.gyroOffset = new Rotation3d(pose.getRotation().minus(this.odometryPose.getRotation().minus(gyroOffsetYaw))).plus(gyroOffsetNoYaw);
        this.estimatedGlobalPose = pose;
        this.odometryPose = pose;
        this.poseBuffer.clear();
    }

    public void addOdometryObservation(OdometryObservation observation) {
        var twist = DriveConstants.kinematics.toTwist2d(observation.startModulePositions(), observation.endModulePositions());
        var lastOdometryPose = this.odometryPose;
        this.odometryPose = this.odometryPose.exp(twist);
        if (observation.gyroRotation.isPresent()) {
            this.odometryPose = new Pose2d(this.odometryPose.getTranslation(), observation.gyroRotation.get().plus(this.gyroOffset).toRotation2d());
        }
        this.poseBuffer.addSample(observation.timestamp(), this.odometryPose);
        var finalTwist = lastOdometryPose.log(this.odometryPose);
        this.estimatedGlobalPose = this.estimatedGlobalPose.exp(finalTwist);
    }

    public void addVisionObservation(VisionObservation observation) {
        try {
            if (this.poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSecs > observation.timestamp()) {
                return;
            }
        } catch (NoSuchElementException e) {
            return;
        }
        var sample = this.poseBuffer.getSample(observation.timestamp());
        if (sample.isEmpty()) {return;}

        var sampleToOdometryTransform = new Transform2d(sample.get(), this.odometryPose);
        var odometryToSampleTransform = new Transform2d(this.odometryPose, sample.get());

        var globalEstimateAtTime = this.estimatedGlobalPose.plus(odometryToSampleTransform);

        var r = new double[3];
        for (int i = 0; i < 3; i++) {
            r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
        }
        var visionK = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; row++) {
            double stdDev = this.qStdDevs.get(row, 0);
            if (stdDev == 0.0) {
                visionK.set(row, row, 0.0);
            } else {
                visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
            }
        }

        var globalEstimateAtTimeToObservation = new Transform2d(globalEstimateAtTime, observation.pose());
        var kTimesTransform = visionK.times(
            VecBuilder.fill(
                globalEstimateAtTimeToObservation.getX(),
                globalEstimateAtTimeToObservation.getY(),
                globalEstimateAtTimeToObservation.getRotation().getRadians()
            )
        );
        var scaledTransform = new Transform2d(
            kTimesTransform.get(0, 0),
            kTimesTransform.get(1, 0),
            Rotation2d.fromRadians(kTimesTransform.get(2, 0))
        );

        this.estimatedGlobalPose = globalEstimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
    }

    public static record OdometryObservation(
        double timestamp,
        Optional<Rotation3d> gyroRotation,
        SwerveModulePosition[] startModulePositions,
        SwerveModulePosition[] endModulePositions
    ) {}
    public static record VisionObservation(
        double timestamp,
        Pose2d pose,
        Matrix<N3, N1> stdDevs
    ) {}
}
