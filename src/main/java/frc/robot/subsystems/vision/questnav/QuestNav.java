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

        Logger.recordOutput("QuestNav/Pose", inputs.pose);
        Logger.recordOutput("QuestNav/RobotPose", getRobotPose());

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
        return inputs.pose.transformBy(new Transform2d(
            camMeta.mount.getRobotRelative().getTranslation().toTranslation2d(),
            camMeta.mount.getRobotRelative().getRotation().toRotation2d()
        ).inverse());
    }

    private Pose2d getRobotPose() {
        return getRobotCenter().transformBy(offset);
    }

    public void resetToPose(Pose2d pose) {
        this.offset = getRobotCenter().minus(pose);
    }
}
