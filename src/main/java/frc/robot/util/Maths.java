package frc.robot.util;

public class Maths {
    /**
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @return Euclidean distance between (x1, y1) and (x2, y2)
     */
    public static double distance(float x1, float y1, float x2, float y2) { 
        return Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    /**
     * @param mu Mean of the distribution
     * @param sigma Standard deviation of the distribution
     * @param x
     * @return
     */
    public static double Gaussian(double mu, double sigma, double x) {       
        return Math.exp(-(Math.pow(mu - x, 2)) / Math.pow(sigma, 2) / 2.0) / Math.sqrt(2.0 * Math.PI * Math.pow(sigma, 2));
    }
}
