package frc.util.loggerUtil.tunables;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.math.controller.PIDController;
import frc.util.PIDConstants;

public class LoggedTunablePID {
    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;

    public LoggedTunablePID(String key, double kP, double kI, double kD) {
        this.kP = new LoggedTunableNumber(key + "/kP", kP);
        this.kI = new LoggedTunableNumber(key + "/kI", kI);
        this.kD = new LoggedTunableNumber(key + "/kD", kD);
    }

    public boolean hasChanged(int hashCode) {
        return LoggedTunableNumber.hasChanged(hashCode, kP, kI, kD);
    }

    public double getKP() {
        return kP.get();
    }
    public double getKI() {
        return kI.get();
    }
    public double getKD() {
        return kD.get();
    }
    public PIDConstants getConstants() {
        return new PIDConstants(getKP(), getKI(), getKD());
    }

    public void update(PIDController pid) {
        pid.setPID(
            getKP(),
            getKI(),
            getKD()
        );
    }
    public void update(SlotConfigs pid) {
        pid
            .withKP(getKP())
            .withKI(getKI())
            .withKD(getKD())
        ;
    }
    public void update(Slot0Configs pid) {
        pid
            .withKP(getKP())
            .withKI(getKI())
            .withKD(getKD())
        ;
    }
    public void update(Slot1Configs pid) {
        pid
            .withKP(getKP())
            .withKI(getKI())
            .withKD(getKD())
        ;
    }
    public void update(Slot2Configs pid) {
        pid
            .withKP(getKP())
            .withKI(getKI())
            .withKD(getKD())
        ;
    }
}
