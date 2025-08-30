package frc.robot.subsystems.drive.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;

public class AutoScore {
    public static boolean isBehindReefPose(Pose2d currentPose, Pose2d target){
        //Translation2d[] robotVerticies = getRotatedRobotVertices(pose);
        return currentPose.relativeTo(target).getX() < 0;
    }

    //abs(y) 
    public static final LoggedTunableNumber xScalar = LoggedTunable.from("Auto Score/xScalar", -2);
    public static final LoggedTunableNumber yScalar = LoggedTunable.from("Auto Score/yScalar", -0.5);

    public static Pose2d getTargetPose(Pose2d currentPose, Pose2d target) {
        var relativePose = currentPose.relativeTo(target);
        return target.transformBy(
            new Transform2d(
                new Translation2d(
                    (Math.max(relativePose.getX(), 0) * xScalar.get()) + (Math.abs(relativePose.getY()) * yScalar.get()),
                    0
                ),
                Rotation2d.kZero
            )
        );
    }
}
