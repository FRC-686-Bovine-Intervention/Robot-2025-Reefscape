package frc.robot.subsystems.vision.questnav;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.questnav.QuestNavConstants.QuestNavCameraConstants;
import frc.util.VirtualSubsystem;
import frc.util.misc.GeomUtil.PoseUtils;

public class QuestNav extends VirtualSubsystem {
    private final QuestNavIO io;
    private final QuestNavIOInputsAutoLogged inputs = new QuestNavIOInputsAutoLogged();

    private final QuestNavCameraConstants camMeta;

    private final Alert notConnectedAlert;
    private final Alert lowBatteryAlert;

    private Pose2d robotResetPose = new Pose2d();
    private Pose2d questGlobalOffset = new Pose2d();

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
            RobotState
                .getInstance()
                .addVisionMeasurement(
                    getRobotPose(),
                    VecBuilder.fill(0.00001, 0.00001, Double.POSITIVE_INFINITY),
                    inputs.timestamp
                );
        }
    }

    private Pose2d getRawPose() {
        return inputs.pose.toPose2d();
    }

    private Pose2d getRawPoseRelativeToReset() {
        return PoseUtils.minus(PoseUtils.plus(getRawPose(), questGlobalOffset), robotResetPose);
    }

    private Pose2d getRobotPose() {        
        var questToRobotTransform = 
            new Transform2d(
                camMeta.mount.getRobotRelative().getTranslation().toTranslation2d(),
                camMeta.mount.getRobotRelative().getRotation().toRotation2d()
            ).inverse();
        var questOffsetPose = new Pose2d().transformBy(questToRobotTransform);
        var questOffsetRelativeToReset = getRawPoseRelativeToReset().minus(questOffsetPose);
        var robotTranslation = robotResetPose.transformBy(questOffsetRelativeToReset).transformBy(questToRobotTransform).getTranslation();
        var robotRotation = getRawPose().rotateBy(questToRobotTransform.getRotation()).getRotation();
        return new Pose2d(robotTranslation, robotRotation);
    }

    public void resetToPose(Pose2d pose) {
        this.robotResetPose = pose;
        this.questGlobalOffset = PoseUtils.minus(pose, getRawPose());
    }
}
