package testclient;

import java.util.Random;

public class MathX {
    private static final Random r = new Random();

    /**
     * @param x1 The first point's x-coordinate
     * @param y1 The first point's y-coordinate
     * @param x2 The second point's x-coordinate
     * @param y2 The second point's y-coordinate
     * @return Euclidean distance between (x1, y1) and (x2, y2)
     */
    public static double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    /**
     * @param mu    Mean of the distribution
     * @param sigma Standard deviation of the distribution
     * @param x
     * @return
     */
    public static double Gaussian(double mu, double sigma, double x) {
        return Math.exp(-(Math.pow(mu - x, 2)) / Math.pow(sigma, 2) / 2.0)
                / Math.sqrt(2.0 * Math.PI * Math.pow(sigma, 2));
    }

    public static double normalDistribution(Random r, double mean, double stddev) {
        return r.nextGaussian(mean, stddev);
    }

    public static double normalDistribution(double mean, double stddev) {
        return r.nextGaussian(mean, stddev);
    }

    /**
     * Constrains <code>x</code> between <code>min</code> and <code>max</code>
     * 
     * @param x   The value to constrain between <code>min</code> and
     *            <code>max</code>
     * @param min The minimum value of <code>x</code>
     * @param max The maximum value of <code>x</code>
     * @return The constrained <code>x</code> value
     */
    public static double clamp(double x, double min, double max) {
        return Math.max(min, Math.min(x, max));
    }

    /**
     * @param p Probability between 0 and 1 of generating a true value
     * @return
     */
    public static boolean bernoulliDistribution(double p) {
        return Math.random() < p;
    }
}
