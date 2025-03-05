package frc.robot.auto;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.RobotConstants;
import frc.util.flipping.AllianceFlipped;

public final class AutoConstants {
    public static final Time allottedAutoTime = Seconds.of(15.3);
    public static final Time disabledTime = Seconds.of(3);

    public static final Distance startLineX = Inches.of(297.490611);
    public static final Distance startX = startLineX.minus(RobotConstants.centerToFrontBumper);
    public static final AllianceFlipped<Pose2d> startFarLeft = 
        AllianceFlipped.fromBlue(new Pose2d(
            startX,
            Reef.pipes[9].robotPose.getBlue().getForward().getMeasureY(),
            Rotation2d.k180deg
        ));
    public static final AllianceFlipped<Pose2d> startRemoteLeft = 
        AllianceFlipped.fromBlue(new Pose2d(
            startX,
            startFarLeft.getBlue().getMeasureY().plus(RobotConstants.centerToFrontBumper.times(3)),
            Rotation2d.k180deg
        ));
    public static final AllianceFlipped<Pose2d> startLeftCage =
        AllianceFlipped.fromBlue(new Pose2d(
            startX,
            Reef.pipes[8].robotPose.getBlue().getForward().getMeasureY(),
            Rotation2d.k180deg
        ));
    public static final AllianceFlipped<Pose2d> startLeftCenter =
        AllianceFlipped.fromBlue(new Pose2d(
            startX,
            Reef.pipes[7].robotPose.getBlue().getForward().getMeasureY(),
            Rotation2d.k180deg
        ));

    public static final AllianceFlipped<Pose2d> startFarRight = 
        AllianceFlipped.fromBlue(new Pose2d(
            startX,
            Reef.pipes[4].robotPose.getBlue().getForward().getMeasureY(),
            Rotation2d.k180deg
        ));
    public static final AllianceFlipped<Pose2d> startRemoteRight = 
        AllianceFlipped.fromBlue(new Pose2d(
            startX,
            startFarRight.getBlue().getMeasureY().minus(RobotConstants.centerToFrontBumper.times(3)),
            Rotation2d.k180deg
        ));
    public static final AllianceFlipped<Pose2d> startRightCage =
        AllianceFlipped.fromBlue(new Pose2d(
            startX,
            Reef.pipes[5].robotPose.getBlue().getForward().getMeasureY(),
            Rotation2d.k180deg
        ));
    public static final AllianceFlipped<Pose2d> startRightCenter =
        AllianceFlipped.fromBlue(new Pose2d(
            startX,
            Reef.pipes[6].robotPose.getBlue().getForward().getMeasureY(),
            Rotation2d.k180deg
        ));
}
