package frc.robot.subsystems.objectiveTracker;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectiveSelectorIO {
    @AutoLog
    public static class ObjectiveSelectorIOInputs {
        public int coral;   // 0 to 47 (coral nodes)
        public int algae;   // 0 (net), 1 (processor), 2 (opponent's processor)
        public int intake;  // 0 (coral station), 1 - 6 (staged algae)
        public int cage;    // 0 (left), 1 (center), 2 (right)
    }

    public default void updateInputs(ObjectiveSelectorIOInputs inputs) {}
    
    public void setCoral(int objective);
    public void setAlgae(int objective);
    public void setIntake(int objective);
    public void setCage (int objective);
}
