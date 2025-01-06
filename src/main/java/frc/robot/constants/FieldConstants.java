package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.util.AllianceFlipUtil.FlippedGeometry;

public final class FieldConstants {
    public static final Distance fieldLength = Inches.of(57*12 + 6 + 7.0/8.0);
    public static final Distance fieldWidth =  Inches.of(26*12 + 5);

    public static final AprilTagFieldLayout apriltagLayout;
    static {
        AprilTagFieldLayout a = null;
        try {
            a = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch(Exception e) {
            e.printStackTrace();
        }
        apriltagLayout = a;
    }

    public static final class Coral {
        public static final Distance length = Inches.of(11.875);
        public static final Distance radius = Inches.of(2);

        public static final Transform3d rackPlacement = new Transform3d(
            new Translation3d(
                length.div(2).minus(Inches.of(2)),
                Inches.zero(),
                Inches.zero()
            ),
            Rotation3d.kZero
        );
        public static final Transform3d standUp = new Transform3d(
            new Translation3d(
                Inches.zero(),
                Inches.zero(),
                length.div(2)
            ),
            new Rotation3d(
                Degrees.zero(),
                Degrees.of(-90),
                Degrees.zero()
            )
        );
    }

    public static final class Algae {
        public static final Distance radius = Inches.of(0);
    }


    public static final class Reef {
        private static final Distance level23Radius = Meters.of(0.779254);
        public static final Angle level23Angle = Degrees.of(35);
        private static final Rotation3d level23Rotation = new Rotation3d(
            Degrees.zero(),
            level23Angle,
            Degrees.zero()
        );

        public static final Distance level2Height = Meters.of(0.792953);
        private static final Transform3d level2 = new Transform3d(
            new Translation3d(
                level23Radius.unaryMinus(),
                Meters.zero(),
                level2Height
            ),
            level23Rotation
        );
        public static final Distance level3Height = Meters.of(1.196053);
        private static final Transform3d level3 = new Transform3d(
            new Translation3d(
                level23Radius.unaryMinus(),
                Meters.zero(),
                level3Height
            ),
            level23Rotation
        );
        public static final Distance level4Height = Meters.of(1.828663);
        private static final Distance level4Radius = Meters.of(0.780750);
        public static final Angle level4Angle = Degrees.of(90);
        private static final Transform3d level4 = new Transform3d(
            new Translation3d(
                level4Radius.unaryMinus(),
                Meters.zero(),
                level4Height
            ),
            new Rotation3d(
                Degrees.zero(),
                level4Angle,
                Degrees.zero()
            )
        );

        private static final Transform3d left = new Transform3d(
            new Translation3d(
                Meters.zero(),
                Meters.of(+0.164308),
                Meters.zero()
            ),
            Rotation3d.kZero
        );
        private static final Transform3d right = new Transform3d(
            new Translation3d(
                Meters.zero(),
                Meters.of(-0.164309),
                Meters.zero()
            ),
            Rotation3d.kZero
        );

        public static final FlippedGeometry<Translation2d> reefCenter = FlippedGeometry.fromBlue(
            new Translation2d(
                Meters.of(4.489325),
                Meters.of(4.025877)
            )
        );
        private static final Rotation2d rackDelta = new Rotation2d(
            Degrees.of(60)
        );
        private static final Pose2d rack0Origin = new Pose2d(
            reefCenter.getBlue(),
            rackDelta.times(0)
        );
        private static final Pose2d rack1Origin = new Pose2d(
            reefCenter.getBlue(),
            rackDelta.times(1)
        );
        private static final Pose2d rack2Origin = new Pose2d(
            reefCenter.getBlue(),
            rackDelta.times(2)
        );
        private static final Pose2d rack3Origin = new Pose2d(
            reefCenter.getBlue(),
            rackDelta.times(3)
        );
        private static final Pose2d rack4Origin = new Pose2d(
            reefCenter.getBlue(),
            rackDelta.times(4)
        );
        private static final Pose2d rack5Origin = new Pose2d(
            reefCenter.getBlue(),
            rackDelta.times(5)
        );

        public static final FlippedGeometry<Pose3d> rack0Level2Left = FlippedGeometry.fromBlue(new Pose3d(rack0Origin).transformBy(level2).transformBy(left));
        public static final FlippedGeometry<Pose3d> rack0Level2Right = FlippedGeometry.fromBlue(new Pose3d(rack0Origin).transformBy(level2).transformBy(right));
        public static final FlippedGeometry<Pose3d> rack0Level3Left = FlippedGeometry.fromBlue(new Pose3d(rack0Origin).transformBy(level3).transformBy(left));
        public static final FlippedGeometry<Pose3d> rack0Level3Right = FlippedGeometry.fromBlue(new Pose3d(rack0Origin).transformBy(level3).transformBy(right));
        public static final FlippedGeometry<Pose3d> rack0Level4Left = FlippedGeometry.fromBlue(new Pose3d(rack0Origin).transformBy(level4).transformBy(left));
        public static final FlippedGeometry<Pose3d> rack0Level4Right = FlippedGeometry.fromBlue(new Pose3d(rack0Origin).transformBy(level4).transformBy(right));

        public static final FlippedGeometry<Pose3d> rack1Level2Left = FlippedGeometry.fromBlue(new Pose3d(rack1Origin).transformBy(level2).transformBy(left));
        public static final FlippedGeometry<Pose3d> rack1Level2Right = FlippedGeometry.fromBlue(new Pose3d(rack1Origin).transformBy(level2).transformBy(right));
        public static final FlippedGeometry<Pose3d> rack1Level3Left = FlippedGeometry.fromBlue(new Pose3d(rack1Origin).transformBy(level3).transformBy(left));
        public static final FlippedGeometry<Pose3d> rack1Level3Right = FlippedGeometry.fromBlue(new Pose3d(rack1Origin).transformBy(level3).transformBy(right));
        public static final FlippedGeometry<Pose3d> rack1Level4Left = FlippedGeometry.fromBlue(new Pose3d(rack1Origin).transformBy(level4).transformBy(left));
        public static final FlippedGeometry<Pose3d> rack1Level4Right = FlippedGeometry.fromBlue(new Pose3d(rack1Origin).transformBy(level4).transformBy(right));

        public static final FlippedGeometry<Pose3d> rack2Level2Left = FlippedGeometry.fromBlue(new Pose3d(rack2Origin).transformBy(level2).transformBy(left));
        public static final FlippedGeometry<Pose3d> rack2Level2Right = FlippedGeometry.fromBlue(new Pose3d(rack2Origin).transformBy(level2).transformBy(right));
        public static final FlippedGeometry<Pose3d> rack2Level3Left = FlippedGeometry.fromBlue(new Pose3d(rack2Origin).transformBy(level3).transformBy(left));
        public static final FlippedGeometry<Pose3d> rack2Level3Right = FlippedGeometry.fromBlue(new Pose3d(rack2Origin).transformBy(level3).transformBy(right));
        public static final FlippedGeometry<Pose3d> rack2Level4Left = FlippedGeometry.fromBlue(new Pose3d(rack2Origin).transformBy(level4).transformBy(left));
        public static final FlippedGeometry<Pose3d> rack2Level4Right = FlippedGeometry.fromBlue(new Pose3d(rack2Origin).transformBy(level4).transformBy(right));
        
        public static final FlippedGeometry<Pose3d> rack3Level2Left = FlippedGeometry.fromBlue(new Pose3d(rack3Origin).transformBy(level2).transformBy(left));
        public static final FlippedGeometry<Pose3d> rack3Level2Right = FlippedGeometry.fromBlue(new Pose3d(rack3Origin).transformBy(level2).transformBy(right));
        public static final FlippedGeometry<Pose3d> rack3Level3Left = FlippedGeometry.fromBlue(new Pose3d(rack3Origin).transformBy(level3).transformBy(left));
        public static final FlippedGeometry<Pose3d> rack3Level3Right = FlippedGeometry.fromBlue(new Pose3d(rack3Origin).transformBy(level3).transformBy(right));
        public static final FlippedGeometry<Pose3d> rack3Level4Left = FlippedGeometry.fromBlue(new Pose3d(rack3Origin).transformBy(level4).transformBy(left));
        public static final FlippedGeometry<Pose3d> rack3Level4Right = FlippedGeometry.fromBlue(new Pose3d(rack3Origin).transformBy(level4).transformBy(right));
        
        public static final FlippedGeometry<Pose3d> rack4Level2Left = FlippedGeometry.fromBlue(new Pose3d(rack4Origin).transformBy(level2).transformBy(left));
        public static final FlippedGeometry<Pose3d> rack4Level2Right = FlippedGeometry.fromBlue(new Pose3d(rack4Origin).transformBy(level2).transformBy(right));
        public static final FlippedGeometry<Pose3d> rack4Level3Left = FlippedGeometry.fromBlue(new Pose3d(rack4Origin).transformBy(level3).transformBy(left));
        public static final FlippedGeometry<Pose3d> rack4Level3Right = FlippedGeometry.fromBlue(new Pose3d(rack4Origin).transformBy(level3).transformBy(right));
        public static final FlippedGeometry<Pose3d> rack4Level4Left = FlippedGeometry.fromBlue(new Pose3d(rack4Origin).transformBy(level4).transformBy(left));
        public static final FlippedGeometry<Pose3d> rack4Level4Right = FlippedGeometry.fromBlue(new Pose3d(rack4Origin).transformBy(level4).transformBy(right));

        public static final FlippedGeometry<Pose3d> rack5Level2Left = FlippedGeometry.fromBlue(new Pose3d(rack5Origin).transformBy(level2).transformBy(left));
        public static final FlippedGeometry<Pose3d> rack5Level2Right = FlippedGeometry.fromBlue(new Pose3d(rack5Origin).transformBy(level2).transformBy(right));
        public static final FlippedGeometry<Pose3d> rack5Level3Left = FlippedGeometry.fromBlue(new Pose3d(rack5Origin).transformBy(level3).transformBy(left));
        public static final FlippedGeometry<Pose3d> rack5Level3Right = FlippedGeometry.fromBlue(new Pose3d(rack5Origin).transformBy(level3).transformBy(right));
        public static final FlippedGeometry<Pose3d> rack5Level4Left = FlippedGeometry.fromBlue(new Pose3d(rack5Origin).transformBy(level4).transformBy(left));
        public static final FlippedGeometry<Pose3d> rack5Level4Right = FlippedGeometry.fromBlue(new Pose3d(rack5Origin).transformBy(level4).transformBy(right));
    }
}
