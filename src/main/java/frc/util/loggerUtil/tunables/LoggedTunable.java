package frc.util.loggerUtil.tunables;

import java.util.function.DoubleFunction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.RobotConstants;

public interface LoggedTunable<T> {
    public static final String TABLE_KEY = "/Tuning";

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    public T get();
    /**
     * Checks whether the value has changed since our last check
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
     *     objects. Recommended approach is to pass the result of "hashCode()"
     * @return True if the value has changed since the last time this method was called, false
     *     otherwise.
     */
    public boolean hasChanged(int id);

    public static boolean hasChanged(int id, LoggedTunable<?>... tunables) {
        if (!RobotConstants.tuningMode) {
            return false;
        }
        var out = false;
        for (var tunable : tunables) {
            if (tunable.hasChanged(id)) {
                out = true;
            }
        }
        return out;
    }

    public static LoggedTunableNumber from(String key, double defaultValue) {
        return new LoggedTunableNumber(key, defaultValue);
    }

    public static <T, U extends Tunable<T>> LoggedTunable<T> from(String key, U defaultValue) {
        return defaultValue.makeTunable(key);
    }

    public static <T> LoggedTunable<T> from(String key, DoubleFunction<T> constructor, double defaultValue) {
        return new LoggedTunable<>() {
            private final LoggedTunableNumber tunableNumber = LoggedTunable.from(key, defaultValue);

            private T cache = constructor.apply(defaultValue);

            @Override
            public boolean hasChanged(int id) {
                return LoggedTunable.hasChanged(id, this.tunableNumber);
            }

            @Override
            public T get() {
                if (this.hasChanged(this.hashCode())) {
                    this.cache = constructor.apply(this.tunableNumber.getAsDouble());
                }
                return this.cache;
            }
        };
    }

    public static LoggedTunable<TrapezoidProfile.Constraints> from(String key, TrapezoidProfile.Constraints defaultValue) {
        return new LoggedTunable<>() {
            private final LoggedTunableNumber maxVelocity = LoggedTunable.from(key + "/Max Velocity", defaultValue.maxVelocity);
            private final LoggedTunableNumber maxAcceleration = LoggedTunable.from(key + "/Max Acceleration", defaultValue.maxAcceleration);

            private TrapezoidProfile.Constraints cache = defaultValue;

            @Override
            public boolean hasChanged(int id) {
                return LoggedTunable.hasChanged(id, this.maxVelocity, this.maxAcceleration);
            }

            @Override
            public TrapezoidProfile.Constraints get() {
                if (this.hasChanged(this.hashCode())) {
                    this.cache = new TrapezoidProfile.Constraints(
                        this.maxVelocity.getAsDouble(),
                        this.maxAcceleration.getAsDouble()
                    );
                }
                return this.cache;
            }
        };
    }

    public static LoggedTunable<Translation2d> from(String key, Translation2d defaultValue) {
        return new LoggedTunable<>() {
            private final LoggedTunableNumber x = LoggedTunable.from(key + "/x", defaultValue.getX());
            private final LoggedTunableNumber y = LoggedTunable.from(key + "/y", defaultValue.getY());

            private Translation2d cache = defaultValue;

            @Override
            public boolean hasChanged(int id) {
                return LoggedTunable.hasChanged(id, this.x, this.y);
            }

            @Override
            public Translation2d get() {
                if (this.hasChanged(this.hashCode())) {
                    this.cache = new Translation2d(this.x.get(), this.y.get());
                }
                return this.cache;
            }
        };
    }
    public static LoggedTunable<Rotation2d> from(String key, Rotation2d defaultValue) {
        return new LoggedTunable<>() {
            private final LoggedTunableNumber theta = LoggedTunable.from(key + "/theta", defaultValue.getRadians());

            private Rotation2d cache = defaultValue;

            @Override
            public boolean hasChanged(int id) {
                return LoggedTunable.hasChanged(id, this.theta);
            }

            @Override
            public Rotation2d get() {
                if (this.hasChanged(this.hashCode())) {
                    this.cache = new Rotation2d(this.theta.get());
                }
                return this.cache;
            }
        };
    }
    public static LoggedTunable<Pose2d> from(String key, Pose2d defaultValue) {
        return new LoggedTunable<>() {
            private final LoggedTunable<Translation2d> translation = LoggedTunable.from(key + "/translation", defaultValue.getTranslation());
            private final LoggedTunable<Rotation2d> rotation = LoggedTunable.from(key + "/rotation", defaultValue.getRotation());

            private Pose2d cache = defaultValue;

            @Override
            public boolean hasChanged(int id) {
                return LoggedTunable.hasChanged(id, this.translation, this.rotation);
            }

            @Override
            public Pose2d get() {
                if (this.hasChanged(this.hashCode())) {
                    this.cache = new Pose2d(this.translation.get(), this.rotation.get());
                }
                return this.cache;
            }
        };
    }
}
