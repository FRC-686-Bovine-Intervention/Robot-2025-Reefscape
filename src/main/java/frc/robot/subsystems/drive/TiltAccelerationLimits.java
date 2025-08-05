package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import frc.util.loggerUtil.tunables.LoggedTunableNumber;

public record TiltAccelerationLimits(
    double forwardLimit,
    double backwardLimit,
    double leftLimit,
    double rightLimit
) {
    public double getMaxTiltAccelerationMPSS(double heading) {
        if (heading < Math.atan2(-this.rightLimit(), -this.backwardLimit())) {
            return -this.backwardLimit() / Math.cos(heading);
        } else if (heading < Math.atan2(-this.rightLimit(), this.forwardLimit())) {
            return -this.rightLimit() / Math.sin(heading);
        } else if (heading < Math.atan2(this.leftLimit(), this.forwardLimit())) {
            return this.forwardLimit() / Math.cos(heading);
        } else if (heading < Math.atan2(this.leftLimit(), -this.backwardLimit())) {
            return this.leftLimit() / Math.sin(heading);
        } else {
            return -this.backwardLimit() / Math.cos(heading);
        }
    }


    public static Supplier<TiltAccelerationLimits> getTunable(String key, TiltAccelerationLimits defaultValue) {
        return new Supplier<TiltAccelerationLimits>() {
            private final LoggedTunableNumber forwardLimit = new LoggedTunableNumber(key + "/Forward", defaultValue.forwardLimit());
            private final LoggedTunableNumber backwardLimit = new LoggedTunableNumber(key + "/Backward", defaultValue.backwardLimit());
            private final LoggedTunableNumber leftLimit = new LoggedTunableNumber(key + "/Left", defaultValue.leftLimit());
            private final LoggedTunableNumber rightLimit = new LoggedTunableNumber(key + "/Right", defaultValue.rightLimit());

            private TiltAccelerationLimits cache = defaultValue;

            @Override
            public TiltAccelerationLimits get() {
                if (LoggedTunableNumber.hasChanged(this.hashCode(), this.forwardLimit, this.backwardLimit, this.leftLimit, this.rightLimit)) {
                    this.cache = new TiltAccelerationLimits(
                        this.forwardLimit.get(),
                        this.backwardLimit.get(),
                        this.leftLimit.get(),
                        this.rightLimit.get()
                    );
                }
                return this.cache;
            }
        };
    }
}
