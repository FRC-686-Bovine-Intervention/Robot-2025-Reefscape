package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.OdometryThread.DoubleBuffer;

public interface OdometryTimestampIO {
    @AutoLog
    public static class OdometryTimestampIOInputs {
        public double[] timestamps = new double[0];
    }

    public default void updateInputs(OdometryTimestampIOInputs inputs) {}

    public static class OdometryTimestampIOOdometryThread implements OdometryTimestampIO {
        private final DoubleBuffer timestampBuffer;

        public OdometryTimestampIOOdometryThread() {
            this.timestampBuffer = OdometryThread.getInstance().generateTimestampBuffer();
        }

        @Override
        public void updateInputs(OdometryTimestampIOInputs inputs) {
            inputs.timestamps = this.timestampBuffer.popAll();
        }
    }
    public static class OdometryTimestampIOSim implements OdometryTimestampIO {
        public OdometryTimestampIOSim() {}

        @Override
        public void updateInputs(OdometryTimestampIOInputs inputs) {
            inputs.timestamps = new double[] {Timer.getTimestamp()};
        }
    }
}
