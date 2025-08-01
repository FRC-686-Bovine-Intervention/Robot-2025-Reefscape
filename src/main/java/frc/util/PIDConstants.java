package frc.util;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.math.controller.PIDController;

public record PIDConstants(double kP, double kI, double kD) {
    public void update(PIDController pid) {
        pid.setPID(
            kP(),
            kI(),
            kD()
        );
    }
    public void update(SlotConfigs pid) {
        pid
            .withKP(kP())
            .withKI(kI())
            .withKD(kD())
        ;
    }
    public void update(Slot0Configs pid) {
        pid
            .withKP(kP())
            .withKI(kI())
            .withKD(kD())
        ;
    }
    public void update(Slot1Configs pid) {
        pid
            .withKP(kP())
            .withKI(kI())
            .withKD(kD())
        ;
    }
    public void update(Slot2Configs pid) {
        pid
            .withKP(kP())
            .withKI(kI())
            .withKD(kD())
        ;
    }
}
