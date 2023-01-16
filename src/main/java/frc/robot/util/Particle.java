package frc.robot.util;

import java.util.Random;

public class Particle {
    public float forwardNoise, turnNoise, senseNoise;
    public float x, y, orientation;
    public float worldWidth, worldHeight;
    public double probability = 0;
    public Point[] landmarks;
    Random random;

    /**
     * Default constructor for a particle
     * 
     * @param landmarks Point array of landmark points for the particle
     * @param worldWidth2 width of the particle's world in pixels
     * @param worldHeight2 height of the particle's world in pixels
     */
    public Particle(Point[] landmarks, float worldWidth2, float worldHeight2) {
        this.landmarks = landmarks;
        this.worldWidth = worldWidth2;
        this.worldHeight = worldHeight2;
        random = new Random();
        x = random.nextFloat() * worldWidth2;
        y = random.nextFloat() * worldHeight2;
        orientation = random.nextFloat() * 2f * ((float)Math.PI);
        forwardNoise = 0f;
        turnNoise = 0f;
        senseNoise = 0f;
    }

    /**
     * Sets the position of the particle and its relative probability
     * 
     * @param x new x position of the particle
     * @param y new y position of the particle
     * @param orientation new orientation of the particle, in radians
     * @param prob new probability of the particle between 0 and 1
     */
    public void set(float x, float y, float orientation, double prob) {
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
    public void setNoise(float Fnoise, float Tnoise, float Snoise) {
        this.forwardNoise = Fnoise;
        this.turnNoise = Tnoise;
        this.senseNoise = Snoise;
    }

    /**
     * Senses the distance of the particle to each of its landmarks
     * 
     * @return a float array of distances to landmarks
     */
    public float[] sense() {
        float[] ret = new float[landmarks.length];
        
        for (int i = 0; i < landmarks.length; ++i){
            float dist = (float) Maths.distance(x, y, landmarks[i].x, landmarks[i].y);
            ret[i] = dist + (float) random.nextGaussian() * senseNoise;
        } 

        return ret;
    }

    /**
     * Moves the particle's position
     * 
     * @param turn turn value, in degrees
     * @param forward move value, must be >= 0
     */
    public void move(float turn, float forward) {
        orientation = orientation + turn + (float) random.nextGaussian() * turnNoise;
        orientation = circle(orientation, 2f * (float) Math.PI);
        
        double dist = forward + random.nextGaussian() * forwardNoise;
        
        x += Math.cos(orientation) * dist;
        y += Math.sin(orientation) * dist;
        x = circle(x, worldWidth);
        y = circle(y, worldHeight); 
    }

    /**
     * Calculates the probability of particle based on another particle's sense()
     * 
     * @param measurement distance measurements from another particle's sense()
     * @return the probability of the particle being correct, between 0 and 1
     */
    public double measurementProb(float[] measurement) {
        double prob = 1.0;
        for(int i = 0; i < landmarks.length; i++) {
            float dist = (float) Maths.distance(x, y, landmarks[i].x, landmarks[i].y);            
            prob *= Maths.Gaussian(dist, senseNoise, measurement[i]);            
        }      
        
        probability = prob;
        
        return prob;
    }

    private float circle(float num, float length) {         
        while (num > length - 1) num -= length;
        while (num < 0) num += length;
        return num;       
    }
    
    @Override
    public String toString() {
        return "[x=" + x + " y=" + y + " orient=" + Math.toDegrees(orientation) + " prob=" +probability +  "]";
    }
}
