package frc.robot.subsystems.objectiveTracker;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectiveSelectorIO {
    @AutoLog
    public static class ObjectiveSelectorIOInputs {
        public int coral;   // 0 to 47 (coral nodes)
        public int algae;   // 0 (net), 1 (processor), 2 (opponent's processor)
        public int intake;  // 0 (coral station), 1 (ground algae), 2 - 7 (staged algae)
        public int cage;
        public int[] branchQueue = new int[0]; // Add coral = id of branch, Remove coral = id of branch-37
        public int[] level1Count = new int[0]; // Value = coral count
        public int[] toggledAlgae = new int[0]; // Add algae = id of algae, Remove algae = id of algae-7
        public boolean[] coop = new boolean[0]; // Value = coop state
        public int[] priorityList = new int[0];
    }

    public default void updateInputs(ObjectiveSelectorIOInputs inputs) {}
    
    public default void setCoral(int objective) {}
    public default void setAlgae(int objective) {}
    public default void setIntake(int objective) {}
    public default void setCage(int objective) {}
}
