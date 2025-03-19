package frc.robot.subsystems.vision.questnav;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.vision.questnav.QuestNavConstants.QuestNavCameraConstants;
import frc.util.VirtualSubsystem;
import frc.util.misc.GeomUtil.PoseUtils;

public class QuestNav extends VirtualSubsystem {
    private final QuestNavIO io;
    private final QuestNavIOInputsAutoLogged inputs = new QuestNavIOInputsAutoLogged();

    private final QuestNavCameraConstants camMeta;

    private final Alert notConnectedAlert;
    private final Alert lowBatteryAlert;

    private Pose3d robotResetPose = new Pose3d();
    private Pose3d questGlobalOffset = new Pose3d();

    public QuestNav(QuestNavCameraConstants camMeta, QuestNavIO io) {
        this.camMeta = camMeta;
        this.io = io;

        this.notConnectedAlert = new Alert("QuestNav camera \"" + camMeta.hardwareName + "\" is not connected", AlertType.kError);
        this.lowBatteryAlert = new Alert("QuestNav camera \"" + camMeta.hardwareName + "\" Battery is Low! (<25%)", AlertType.kWarning);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/QuestNav/" + camMeta.hardwareName, inputs);
        io.cleanUp();

        Logger.recordOutput("QuestNav/RawPose", getRawPose());
        Logger.recordOutput("QuestNav/RawPoseRelativeToReset", getRawPoseRelativeToReset());
        Logger.recordOutput("QuestNav/RobotPose", getRobotPose());

        notConnectedAlert.set(!inputs.isConnected);
        lowBatteryAlert.set(inputs.isConnected && inputs.batteryPercent < 25);

        if (DriverStation.isDisabled()) {
            resetToPose(new Pose2d(4, 2, new Rotation2d(0)));
        } else {
            // RobotState
            //     .getInstance()
            //     .addVisionMeasurement(
            //         getRobotPose().toPose2d(),
            //         VecBuilder.fill(0.00001, 0.00001, Double.POSITIVE_INFINITY),
            //         inputs.timestamp
            //     );
        }
    }

    private Pose3d getRawPose() {
        return inputs.pose;
    }

    private Pose3d getRawPoseRelativeToReset() {
        return PoseUtils.minus(PoseUtils.plus(getRawPose(), questGlobalOffset), robotResetPose);
    }

    private Pose3d getRobotPose() {        
        return PoseUtils.plus(getRawPoseRelativeToReset(), robotResetPose).plus(camMeta.mount.getRobotRelative().inverse());
    }

    public void resetToPose(Pose2d pose) {
        resetToPose(new Pose3d(pose));
    }

    public void resetToPose(Pose3d pose) {
        this.robotResetPose = pose;
        this.questGlobalOffset = PoseUtils.minus(pose, getRawPose());
    }
}
