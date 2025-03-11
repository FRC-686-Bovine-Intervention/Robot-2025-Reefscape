package frc.robot.subsystems.vision.questnav;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.questnav.QuestNavConstants.QuestNavCameraConstants;
import frc.util.VirtualSubsystem;

public class QuestNav extends VirtualSubsystem {
    private final QuestNavIO io;
    private final QuestNavIOInputsAutoLogged inputs = new QuestNavIOInputsAutoLogged();

    private final QuestNavCameraConstants camMeta;

    private Transform2d offset = new Transform2d();

    private final Alert notConnectedAlert;

    public QuestNav(QuestNavCameraConstants camMeta, QuestNavIO io) {
        this.camMeta = camMeta;
        this.io = io;

        notConnectedAlert = new Alert("QuestNav camera \"" + camMeta.hardwareName + "\" is not connected", AlertType.kError);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/QuestNav/" + camMeta.hardwareName, inputs);
        io.cleanUp();

        Logger.recordOutput("QuestNav/Mount", camMeta.mount.getFieldRelative());

        Logger.recordOutput("QuestNav/RawPose", inputs.pose);
        Logger.recordOutput("QuestNav/RobotCenter", getRobotCenter());
        Logger.recordOutput("QuestNav/OffsetPose", getRobotPose());

        notConnectedAlert.set(!inputs.isConnected);

        if (DriverStation.isDisabled()) {
            resetToPose(RobotState.getInstance().getPose());
        } else {
            // RobotState
            //     .getInstance()
            //     .addVisionMeasurement(
            //         getRobotPose(),
            //         VecBuilder.fill(0.00001, 0.00001, Double.POSITIVE_INFINITY),
            //         inputs.timestamp
            //     );
        }
    }

    private Pose2d getRobotCenter() {
        return new Pose2d(
            inputs.pose.getTranslation().minus(camMeta.mount.getRobotRelative().getTranslation().toTranslation2d()),
            inputs.pose.getRotation().minus(camMeta.mount.getRobotRelative().getRotation().toRotation2d())
        );
    }

    private Pose2d getRobotPose() {
        return new Pose2d(
            getRobotCenter().getTranslation().plus(offset.getTranslation()),
            getRobotCenter().getRotation().plus(offset.getRotation())
        );
    }

    public void resetToPose(Pose2d pose) {
        this.offset = new Transform2d(pose.getTranslation().minus(getRobotCenter().getTranslation()), pose.getRotation().minus(getRobotCenter().getRotation()));
    }
}
