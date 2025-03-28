package frc.robot.subsystems.objectiveTracker;

import org.littletonrobotics.junction.AutoLog;

public interface ReefTrackerIO {
    @AutoLog
    public static class ReefTrackerIOInputs {
        public int coralGoal = 0;
        public int algaeGoal = 0;

        public int selectedLevel = 0;
        public int level1State = 0;
        public int level2State = 0;
        public int level3State = 0;
        public int level4State = 0;
        public int algaeState = 0;
        public boolean coopState = false;
    }

    public default void updateInputs(ReefTrackerIOInputs inputs) {}


    default void setCoralGoal(int value) {}
    default void setAlgaeGoal(int value) {}
    
    default void setSelectedLevel(int value) {}
    default void setLevel1State(int value) {}
    default void setLevel2State(int value) {}
    default void setLevel3State(int value) {}
    default void setLevel4State(int value) {}
    default void setAlgaeState(int value) {}
    default void setCoopState(boolean value) {}
}
