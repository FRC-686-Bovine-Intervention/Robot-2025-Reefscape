package frc.util.misc;

import java.util.Arrays;

public class MathExtraUtil {
    public static double average(double... a) {
        return Arrays.stream(a).average().orElse(0);
    }

    public static boolean isWithin(double value, double min, double max) {
        return value >= min && value <= max;
    }
}
