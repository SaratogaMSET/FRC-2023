package frc.lib.util;

public class MathUtils {
    public static boolean equalsWithinError(double targetValue, double currentValue, double error) {
        return Math.abs(currentValue - targetValue) <= error;
    }

    public static double ensureRange(double value, double minValue, double maxValue) {
        return Math.max(minValue, Math.min(value, maxValue));
    }


    /**
     * @param value
     * @param minValue
     * @param maxValue
     * @return Whether or not <code>value</code> is in the range [<code>minValue</code>, <code>maxValue</code>)
     */
    public static boolean isInRange(double value, double minValue, double maxValue) {
        return minValue <= value && value < maxValue;
    }
}