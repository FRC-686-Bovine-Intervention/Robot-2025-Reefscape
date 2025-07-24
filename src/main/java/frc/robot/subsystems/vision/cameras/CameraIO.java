package frc.robot.subsystems.vision.cameras;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.vision.Pipeline;

public interface CameraIO {
    public class CameraIOInputs implements LoggableInputs {
        public boolean isConnected;
        public LoggableInputs[] pipelines;

        public CameraIOInputs(LoggableInputs... pipelineInputs) {
            this.isConnected = false;
            this.pipelines = pipelineInputs;
        }

        @Override
        public void toLog(LogTable table) {
            table.put("IsConnected", this.isConnected);
            var pipelineTable = table.getSubtable("Pipelines");
            for (int i = 0; i < this.pipelines.length; i++) {
                this.pipelines[i].toLog(pipelineTable.getSubtable(Integer.toString(i)));
            }
        }

        @Override
        public void fromLog(LogTable table) {
            this.isConnected = table.get("IsConnected", this.isConnected);
            var pipelineTable = table.getSubtable("Pipelines");
            for (int i = 0; i < this.pipelines.length; i++) {
                this.pipelines[i].fromLog(pipelineTable.getSubtable(Integer.toString(i)));
            }
        }
    }
    

    public default void updateInputs(CameraIOInputs inputs, Pipeline[] pipelines) {}

    public default void setPipeline(int pipelineIndex) {}

    public default void setDriverMode(boolean driverMode) {}

    // TODO
    public default void setLEDMode() {}
}
