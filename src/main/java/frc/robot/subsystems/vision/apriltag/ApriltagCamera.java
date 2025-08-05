package frc.robot.subsystems.vision.apriltag;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.vision.apriltag.ApriltagCameraIO.ApriltagCameraFrame;
import frc.robot.subsystems.vision.apriltag.ApriltagCameraIO.ApriltagCameraIOInputs;
import frc.robot.subsystems.vision.apriltag.ApriltagVisionConstants.ApriltagCameraConstants;
import frc.util.LoggedTracer;
import frc.util.led.animation.StatusLightAnimation;

public class ApriltagCamera {
    private final ApriltagCameraConstants camMeta;
    private final ApriltagCameraIO io;
    private final ApriltagCameraIOInputs inputs = new ApriltagCameraIOInputs();

    private final StatusLightAnimation connectionAnimation;
    private final Alert notConnectedAlert;

    public ApriltagCamera(ApriltagCameraConstants camMeta, ApriltagCameraIO io, StatusLightAnimation connectionAnimation) {
        this.camMeta = camMeta;
        this.io = io;
        this.connectionAnimation = connectionAnimation;

        this.notConnectedAlert = new Alert("Apriltag camera \"" + camMeta.hardwareName + "\" is not connected", AlertType.kError);
    }

    public ApriltagCameraResult periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/ApriltagVision/" + camMeta.hardwareName, inputs);
        LoggedTracer.logEpoch("VirtualSubsystem Periodic/ApriltagVision/Camera Periodic/" + camMeta.hardwareName + "/Process Inputs");

        notConnectedAlert.set(!inputs.isConnected);
        connectionAnimation.setStatus(inputs.isConnected);

        LoggedTracer.logEpoch("VirtualSubsystem Periodic/ApriltagVision/Camera Periodic/" + camMeta.hardwareName);
        return ApriltagCameraResult.from(camMeta, inputs);
    }

    public static class ApriltagCameraResult {
        public final ApriltagCameraConstants camMeta;
        public final ApriltagCameraFrame[] frames;

        private ApriltagCameraResult(ApriltagCameraConstants camMeta, ApriltagCameraFrame[] frames) {
            this.camMeta = camMeta;
            this.frames = frames;
        }

        public static ApriltagCameraResult from(ApriltagCameraConstants camMeta, ApriltagCameraIOInputs inputs) {
            if (!inputs.isConnected) {
                return new ApriltagCameraResult(camMeta, new ApriltagCameraFrame[0]);
            }
            return new ApriltagCameraResult(
                camMeta,
                inputs.frames
            );
        }
    }
}