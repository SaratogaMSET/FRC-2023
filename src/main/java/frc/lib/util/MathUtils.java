package frc.lib.util;

public class MathUtils {
    public static boolean equalsWithinError(double targetValue, double currentValue, double error) {
        return Math.abs(currentValue - targetValue) <= error;
    }

    public static double ensureRange(double value, double minValue, double maxValue) {
        return Math.max(minValue, Math.min(value, maxValue));
    }

    public static boolean isInRange(double value, double minValue, double maxValue) {
        return minValue <= value && value <= maxValue;
    }
}