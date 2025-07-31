package frc.robot.subsystems.manualOverrides;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class ManualOverrides {
    private final LoggedNetworkBoolean disableSelfRecordCoral = new LoggedNetworkBoolean("Manual Overrides/Objective Tracker/Disable Self Record Coral", false);
    private final LoggedNetworkBoolean disableSelfRecordAlgae = new LoggedNetworkBoolean("Manual Overrides/Objective Tracker/Disable Self Record Algae", false);
    private final LoggedNetworkBoolean disableAutoEjectCoral = new LoggedNetworkBoolean("Manual Overrides/Auto Eject/Disable Auto Eject Coral", false);
    
    public ManualOverrides() {

    }

    public boolean selfRecordCoralDisabled() {
        return this.disableSelfRecordCoral.get();
    }
    public boolean selfRecordAlgaeDisabled() {
        return this.disableSelfRecordAlgae.get();
    }
    public boolean autoEjectCoralDisabled() {
        return this.disableAutoEjectCoral.get();
    }
}
