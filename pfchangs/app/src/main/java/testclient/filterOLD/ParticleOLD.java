package testclient.filterOLD;

import java.util.Random;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import testclient.Maths;

public class ParticleOLD {
    public double forwardNoise, turnNoise, senseNoise;
    public double x, y, orientation;
    public double worldWidth, worldHeight;
    public double probability = 0;
    public org.opencv.core.Point3[] landmarks;
    Random random;

    /**
     * Default constructor for a particle
     * 
     * @param landmarks    Point array of landmark points for the particle
     * @param worldWidth2  width of the particle's world in pixels
     * @param worldHeight2 height of the particle's world in pixels
     */
    public ParticleOLD(org.opencv.core.Point3[] landmarks, double worldWidth2, double worldHeight2) {
        this.landmarks = landmarks;
        this.worldWidth = worldWidth2;
        this.worldHeight = worldHeight2;
        random = new Random();
        x = random.nextDouble() * worldWidth2;
        y = random.nextDouble() * worldHeight2;
        orientation = random.nextDouble() * 2f * ((double) Math.PI);
        forwardNoise = 0f;
        turnNoise = 0f;
        senseNoise = 0f;
    }

    /**
     * Sets the position of the particle and its relative probability
     * 
     * @param x           new x position of the particle
     * @param y           new y position of the particle
     * @param orientation new orientation of the particle, in radians
     * @param prob        new probability of the particle between 0 and 1
     */
    public void set(double x, double y, double orientation, double prob) {
        this.x = x;
        this.y = y;
        this.orientation = orientation;
        this.probability = prob;
    }

    /**
     * Sets the noise of the particles measurements and movements
     * 
     * @param Fnoise noise of particle in forward movement
     * @param Tnoise noise of particle in turning
     * @param Snoise noise of particle in sensing position
     */
    public void setNoise(double Fnoise, double Tnoise, double Snoise) {
        this.forwardNoise = Fnoise;
        this.turnNoise = Tnoise;
        this.senseNoise = Snoise;
    }

    /**
     * Senses the distance of the particle to each of its landmarks
     * 
     * @return a double array of distances to landmarks
     */
    public double[] sense() {
        double[] ret = new double[landmarks.length];

        for (int i = 0; i < landmarks.length; ++i) {
            double dist = (double) Maths.distance(x, y, landmarks[i].x, landmarks[i].y);
            ret[i] = dist + (double) random.nextGaussian() * senseNoise;
        }

        return ret;
    }

    /**
     * Moves the particle's position
     * 
     * @param turn    turn value, in degrees
     * @param forward move value, must be >= 0
     */
    public void move(double turn, double forward) {
        orientation = orientation + turn + (double) random.nextGaussian() * turnNoise;
        orientation = circle(orientation, 2f * (double) Math.PI);

        double dist = forward + random.nextGaussian() * forwardNoise;

        x += Math.cos(orientation) * dist;
        y += Math.sin(orientation) * dist;
        x = circle(x, worldWidth);
        y = circle(y, worldHeight);
    }

    public void move(double xTrans, double yTrans, double turn) {
        orientation = orientation + turn + (double) random.nextGaussian() * turnNoise;
        orientation = circle(orientation, 2f * (double) Math.PI);
        x += xTrans + random.nextGaussian() * forwardNoise;
        y += yTrans + random.nextGaussian() * forwardNoise;
    }

    /**
     * Calculates the probability of particle based on another particle's sense()
     * 
     * @param measurement distance measurements from another particle's sense()
     * @return the probability of the particle being correct, between 0 and 1
     */
    public double measurementProb(double[] measurement) {
        double prob = 1.0;
        for (int i = 0; i < landmarks.length; i++) {
            if (measurement[i] > 0) {
                double dist = (double) Maths.distance(x, y, landmarks[i].x, landmarks[i].y);
                prob *= Maths.Gaussian(dist, senseNoise, measurement[i]);
            }
        }

        probability = prob;

        return prob;
    }

    public Pose2d toPose2d() {
        return new Pose2d(
            new Translation2d(x, y),
            new Rotation2d(orientation)
        );
    }

    public Pose2d toPose2d(double xOffset, double yOffset, double radOffset) {
        return new Pose2d(
            new Translation2d(x + xOffset, y + yOffset),
            new Rotation2d(orientation + radOffset)
        );
    }

    private double circle(double num, double length) {
        while (num > length - 1)
            num -= length;
        while (num < 0)
            num += length;
        return num;
    }

    @Override
    public String toString() {
        return "[x=" + x + " y=" + y + " orient=" + Math.toDegrees(orientation) + " prob=" + probability + "]";
    }
}
