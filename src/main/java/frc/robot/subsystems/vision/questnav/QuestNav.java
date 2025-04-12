package frc.robot.subsystems.vision.questnav;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.questnav.QuestNavConstants.QuestNavCameraConstants;
import frc.util.VirtualSubsystem;
import frc.util.geometry.GeomUtil.TransformUtil;
import frc.util.geometry.RollingAveragePose2d;
import frc.util.led.animation.StatusLightAnimation;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;

public class QuestNav extends VirtualSubsystem {
    private final QuestNavIO io;
    private final QuestNavIOInputsAutoLogged inputs = new QuestNavIOInputsAutoLogged();

    private final QuestNavCameraConstants camMeta;

    private final Alert notConnectedAlert;
    private final Alert lowBatteryAlert;
    private final StatusLightAnimation connectionAnimation;

    private Pose2d questResetPose = new Pose2d();
    private Pose2d robotResetPose = new Pose2d();

    private final RollingAveragePose2d rollingAvg;

    public final LoggedNetworkBoolean isDisabled = new LoggedNetworkBoolean("QuestNav/Quest Disabled");
    public static final LoggedTunableNumber xySTDevs = new LoggedTunableNumber("QuestNav/XY STDevs", 0.1);

    public QuestNav(QuestNavCameraConstants camMeta, QuestNavIO io, StatusLightAnimation connectionAnimation) {
        this.camMeta = camMeta;
        this.io = io;
        this.connectionAnimation = connectionAnimation;
        
        this.rollingAvg = new RollingAveragePose2d(2);

        this.notConnectedAlert = new Alert("QuestNav camera \"" + camMeta.hardwareName + "\" is not connected", AlertType.kError);
        this.lowBatteryAlert = new Alert("QuestNav camera \"" + camMeta.hardwareName + "\" Battery is Low! (<25%)", AlertType.kWarning);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/QuestNav/" + camMeta.hardwareName, inputs);
        io.cleanUp();

        notConnectedAlert.set(!inputs.isConnected);
        lowBatteryAlert.set(inputs.isConnected && inputs.batteryPercent < 25);
        connectionAnimation.setStatus(inputs.isConnected);

        if (!calibrationInProgress && DriverStation.isDisabled()) {
            setPose(RobotState.getInstance().getPose());
        } else if (!calibrationInProgress && inputs.isConnected && !isDisabled.get()) {
            RobotState.getInstance()
                .addVisionMeasurement(
                    getRobotPose(),
                    VecBuilder.fill(xySTDevs.get(), xySTDevs.get(), Double.POSITIVE_INFINITY),
                    inputs.timestamp
                )
            ;
        }

        rollingAvg.addPose(getRobotPose());

        Logger.recordOutput("QuestNav/RawPose", getRawPose());
        Logger.recordOutput("QuestNav/QuestPose", getQuestPose());
        Logger.recordOutput("QuestNav/RobotPose", getRobotPose());
        Logger.recordOutput("QuestNav/AverageRobotPose", getAverageRobotPose());
        Logger.recordOutput("QuestNav/Calibration In Progress", calibrationInProgress);
    }

    public void setPose(Pose2d pose) {
        rollingAvg.reset();
        this.questResetPose = getRawPose();
        this.robotResetPose = pose;
    }

    private Pose2d getRawPose() {
        return inputs.pose.toPose2d();
    }

    public Pose2d getQuestPose() {
        var rawPose = getRawPose();
        var robotToQuest = TransformUtil.toTransform2d(camMeta.mount.getRobotRelative());
        var poseRelativeToReset = rawPose.minus(this.questResetPose);
    
        return robotResetPose
            .transformBy(robotToQuest)
            .transformBy(poseRelativeToReset);
    }

    public Pose2d getRobotPose() {
        var robotToQuest = TransformUtil.toTransform2d(camMeta.mount.getRobotRelative());
        return getQuestPose().transformBy(robotToQuest.inverse());
    }

    public Pose2d getAverageRobotPose() {
        return rollingAvg.getAveragePose();
    }

    private Translation2d calculatedOffsetToRobotCenter = new Translation2d();
    private int calculatedOffsetToRobotCenterCount = 0;
    private boolean calibrationInProgress = false;

    private Translation2d calculateOffsetToRobotCenter() {
        Pose2d currentPose = getRobotPose();

        Rotation2d angle = currentPose.getRotation();
        Translation2d displacement = currentPose.getTranslation();

        double x = ((angle.getCos() - 1) * displacement.getX() + angle.getSin() * displacement.getY())
                / (2 * (1 - angle.getCos()));
        double y = ((-1 * angle.getSin()) * displacement.getX() + (angle.getCos() - 1) * displacement.getY())
                / (2 * (1 - angle.getCos()));

        return new Translation2d(x, y);
    }

    public Command determineOffsetToRobotCenter(Drive drive) {
        return
            Commands.repeatingSequence(
                Commands.run(
                    () -> {
                        drive.rotationalSubsystem.driveVelocity(new ChassisSpeeds(0, 0, 0.314));
                    },
                    drive.rotationalSubsystem).withTimeout(0.5),
                Commands.runOnce(() -> {
                    Translation2d offset = calculateOffsetToRobotCenter();

                    calculatedOffsetToRobotCenter = calculatedOffsetToRobotCenter
                            .times((double) calculatedOffsetToRobotCenterCount
                                    / (calculatedOffsetToRobotCenterCount + 1))
                            .plus(offset.div(calculatedOffsetToRobotCenterCount + 1));
                    calculatedOffsetToRobotCenterCount++;

                    Logger.recordOutput("QuestNav/Calculated Offset to Robot Center", calculatedOffsetToRobotCenter);
                }).onlyIf(() -> getRobotPose().getRotation().getMeasure().in(Degrees) > 30)
            ).beforeStarting(() -> {
                calibrationInProgress = true;
                calculatedOffsetToRobotCenterCount = 0;
                calculatedOffsetToRobotCenter = new Translation2d();
                setPose(Pose2d.kZero);
            }).finallyDo(() -> {
                calibrationInProgress = false;
            });
    }
}
