package testclient.filter;

import java.util.Random;

public class ParticleFilter {
    Particle[] particles;
    int numParticles = 0;
    Random gen = new Random();

    public ParticleFilter(int numParticles, Point[] landmarks, double width, double height) {
        this.numParticles = numParticles;

        particles = new Particle[numParticles];
        for (int i = 0; i < numParticles; i++) {
            particles[i] = new Particle(landmarks, width, height);
        }
    }

    public void setNoise(double Fnoise, double Tnoise, double Snoise) {
        for (int i = 0; i < numParticles; i++) {
            particles[i].setNoise(Fnoise, Tnoise, Snoise);
        }
    }

    public void move(double turn, double forward) {
        for (int i = 0; i < numParticles; i++) {
            particles[i].move(turn, forward);
        }
    }

    public void move(double xTrans, double yTrans, double turn) {
        for (int i = 0; i < numParticles; ++i) {
            particles[i].move(xTrans, yTrans, turn);
        }
    }

    public void resample(double[] measurement) {
        Particle[] new_particles = new Particle[numParticles];

        // Update each particle's probability
        for (int i = 0; i < numParticles; i++) {
            particles[i].measurementProb(measurement);
        }

        double B = 0f;
        Particle best = getBestParticle();
        int index = (int) gen.nextDouble() * numParticles;
        for (int i = 0; i < numParticles; i++) {
            B += gen.nextDouble() * 2f * best.probability;
            while (B > particles[index].probability) {
                B -= particles[index].probability;
                index = circle(index + 1, numParticles);
            }
            new_particles[i] = new Particle(particles[index].landmarks, particles[index].worldWidth, particles[index].worldHeight);
            new_particles[i].set(particles[index].x, particles[index].y, particles[index].orientation, particles[index].probability);
            new_particles[i].setNoise(particles[index].forwardNoise, particles[index].turnNoise, particles[index].senseNoise);
        }

        particles = new_particles;
    }

    private int circle(int num, int length) {
        while (num > length - 1) {
            num -= length;
        }

        while (num < 0) {
            num += length;
        }

        return num;
    }   
    
    public Particle getBestParticle() {
        Particle particle = particles[0];
        for (int i = 0; i < numParticles; i++) {
            if (particles[i].probability > particle.probability) {
                particle = particles[i];
            }
        }

        return particle;
    }
    
    public Particle getAverageParticle() {
        Particle p = new Particle(particles[0].landmarks, particles[0].worldWidth, particles[0].worldHeight);
        double x = 0, y = 0, orient = 0, prob = 0;
        for(int i = 0; i < numParticles; i++) {
            x += particles[i].x;
            y += particles[i].y;
            orient += particles[i].orientation;
            prob += particles[i].probability;
        }
        x /= numParticles;
        y /= numParticles;
        orient /= numParticles;
        prob /= numParticles;
        p.set(x, y, orient, prob);
        
        p.setNoise(particles[0].forwardNoise, particles[0].turnNoise, particles[0].senseNoise);
        
        return p;
    }

    @Override
    public String toString() {
        String res = "";
        for (int i = 0; i < numParticles; i++) {
            res += particles[i].toString() + "\n";
        }
        return res;
    }
}
