package testclient.filterOLD;

import java.util.Random;

import org.opencv.core.Point;

public class ParticleFilterOLD {
    ParticleOLD[] particles;
    int numParticles = 0;
    Random gen = new Random();

    public ParticleFilterOLD(int numParticles, Point[] landmarks, double width, double height) {
        this.numParticles = numParticles;

        particles = new ParticleOLD[numParticles];
        for (int i = 0; i < numParticles; i++) {
            particles[i] = new ParticleOLD(landmarks, width, height);
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
        ParticleOLD[] new_particles = new ParticleOLD[numParticles];

        // Update each particle's probability
        for (int i = 0; i < numParticles; i++) {
            particles[i].measurementProb(measurement);
        }

        double B = 0f;
        ParticleOLD best = getBestParticle();
        int index = (int) gen.nextDouble() * numParticles;
        for (int i = 0; i < numParticles; i++) {
            B += gen.nextDouble() * 2f * best.probability;
            while (B > particles[index].probability) {
                B -= particles[index].probability;
                index = circle(index + 1, numParticles);
            }
            new_particles[i] = new ParticleOLD(particles[index].landmarks, particles[index].worldWidth, particles[index].worldHeight);
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
    
    public ParticleOLD getBestParticle() {
        ParticleOLD particle = particles[0];
        for (int i = 0; i < numParticles; i++) {
            if (particles[i].probability > particle.probability) {
                particle = particles[i];
            }
        }

        return particle;
    }
    
    public ParticleOLD getAverageParticle() {
        ParticleOLD p = new ParticleOLD(particles[0].landmarks, particles[0].worldWidth, particles[0].worldHeight);
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
