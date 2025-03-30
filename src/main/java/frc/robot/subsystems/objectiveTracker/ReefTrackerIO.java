package frc.robot.subsystems.objectiveTracker;

import org.littletonrobotics.junction.AutoLog;

public interface ReefTrackerIO {
    @AutoLog
    public static class ReefTrackerIOInputs {
        public int coralGoal;                                           // 0 to 47 (coral nodes) // id of the branch
        public int algaeGoal;                                           // 0 (net), 1 (processor), 2 (opponent's processor)

        public int[] branchQueue = new int[0];                          // Add coral = id of branch, Remove coral = id of branch-36
        public int[] level1Queue = new int[0];                     // Additional coral count in trough
        public int[] algaeQueue = new int[0];                           // Add algae = id of algae,  Remove algae = id of algae-6
        public int[][] priorityListQueue = new int[0][0];
        public boolean coop = false;                                    // coop state

        public int mode = 0;                                            // 0 = smart, 1 = dumb
    }

    public default void updateInputs(ReefTrackerIOInputs inputs) {}


    default void setMode(int value) {}
    default void setCoralGoal(int value) {}
    default void setAlgaeGoal(int value) {}
    
    default void setCoralState(boolean[] value) {}
    default void setLevel1Count(int value) {}
    default void setAlgaeState(boolean[] value) {}
    default void setCoopState(boolean value) {}
    default void setPriorityList(int[] value) {}
}
