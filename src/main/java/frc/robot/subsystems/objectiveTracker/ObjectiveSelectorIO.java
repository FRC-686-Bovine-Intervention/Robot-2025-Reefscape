package frc.robot.subsystems.objectiveTracker;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectiveSelectorIO {
    @AutoLog
    public static class ObjectiveSelectorIOInputs {
        public int coral;   // 0 to 47 (coral nodes)
        public int algae;   // 0 (coral station), 1 (net), 2 (processor), 3 (opponent's processor)
        public int intake;  // 0 or 5 (staged algae) and 6 (coral station)
    }

    public default void updateInputs(ObjectiveSelectorIOInputs inputs) {}
    
    public void setCoral(int objective);
    public void setAlgae(int objective);
    public void setIntake(int objective);
}
