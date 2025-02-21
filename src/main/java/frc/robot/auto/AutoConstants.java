package frc.robot.auto;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.RobotConstants;
import frc.util.flipping.Flipped;

public final class AutoConstants {
    public static final Time allottedAutoTime = Seconds.of(15.3);
    public static final Time disabledTime = Seconds.of(3);

    public static final Distance startLineX = Inches.of(297.490611);
    public static final Distance startX = startLineX.minus(RobotConstants.centerToFrontBumper);
    public static final Flipped<Pose2d> startFarLeft = 
        Flipped.fromBlue(new Pose2d(
            startX,
            Reef.branches[9].pipe.robotPose.getBlue().getMeasureY(),
            Rotation2d.k180deg
        ));
    public static final Flipped<Pose2d> startLeftCage =
        Flipped.fromBlue(new Pose2d(
            startX,
            Reef.branches[8].pipe.robotPose.getBlue().getMeasureY(),
            Rotation2d.k180deg
        ));
    public static final Flipped<Pose2d> startLeftCenter =
        Flipped.fromBlue(new Pose2d(
            startX,
            Reef.branches[7].pipe.robotPose.getBlue().getMeasureY(),
            Rotation2d.k180deg
        ));

    public static final Flipped<Pose2d> startRightCenter =
        Flipped.fromBlue(new Pose2d(
            startX,
            Reef.branches[6].pipe.robotPose.getBlue().getMeasureY(),
            Rotation2d.k180deg
        ));
    public static final Flipped<Pose2d> startRightCage =
        Flipped.fromBlue(new Pose2d(
            startX,
            Reef.branches[5].pipe.robotPose.getBlue().getMeasureY(),
            Rotation2d.k180deg
        ));
    public static final Flipped<Pose2d> startFarRight = 
        Flipped.fromBlue(new Pose2d(
            startX,
            Reef.branches[4].pipe.robotPose.getBlue().getMeasureY(),
            Rotation2d.k180deg
        ));
}
