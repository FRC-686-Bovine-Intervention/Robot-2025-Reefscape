package frc.util.loggerUtil.tunables;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class LoggedTunableFF {
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kG;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;

    public LoggedTunableFF(String key, double kS, double kG, double kV, double kA) {
        this.kS = new LoggedTunableNumber(key + "/kS", kS);
        this.kG = new LoggedTunableNumber(key + "/kG", kG);
        this.kV = new LoggedTunableNumber(key + "/kV", kV);
        this.kA = new LoggedTunableNumber(key + "/kA", kA);
    }

    public boolean hasChanged(int hashCode) {
        return LoggedTunableNumber.hasChanged(hashCode, kS, kG, kV, kA);
    }

    public double getKS() {
        return kS.get();
    }
    public double getKG() {
        return kG.get();
    }
    public double getKV() {
        return kV.get();
    }
    public double getKA() {
        return kA.get();
    }

    public void update(SimpleMotorFeedforward ff) {
        ff.setKs(getKS());
        ff.setKv(getKV());
        ff.setKa(getKA());
    }
    public void update(ArmFeedforward ff) {
        ff.setKs(getKS());
        ff.setKg(getKG());
        ff.setKv(getKV());
        ff.setKa(getKA());
    }
    public void update(ElevatorFeedforward ff) {
        ff.setKs(getKS());
        ff.setKg(getKG());
        ff.setKv(getKV());
        ff.setKa(getKA());
    }

    public void update(SlotConfigs ff) {
        ff
            .withKS(getKS())
            .withKG(getKG())
            .withKV(getKV())
            .withKA(getKA())
        ;
    }
    public void update(Slot0Configs ff) {
        ff
            .withKS(getKS())
            .withKG(getKG())
            .withKV(getKV())
            .withKA(getKA())
        ;
    }
    public void update(Slot1Configs ff) {
        ff
            .withKS(getKS())
            .withKG(getKG())
            .withKV(getKV())
            .withKA(getKA())
        ;
    }
    public void update(Slot2Configs ff) {
        ff
            .withKS(getKS())
            .withKG(getKG())
            .withKV(getKV())
            .withKA(getKA())
        ;
    }
}
