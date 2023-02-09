package testclient.filter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import org.opencv.core.Point3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import testclient.Constants;
import testclient.Maths;
import testclient.wrappers.TagDistance;

// FIXME check if EVERYTHING (besides LL campose/botpose) is in radians
public class AMCL {
    private ArrayList<TagDistance> tagDistances = new ArrayList<>();

    private Point3 motionDelta = new Point3(); // robot frame motion odometry <-- maybe not?

    // private Particle[] particles;
    private ArrayList<Particle> particles = new ArrayList<Particle>();
    private Particle bestEstimate = new Particle(0, 0, 0, 0);
    private Particle meanEstimate = new Particle(0, 0, 0, 0);

    private double mGaussX, mGaussY, mGaussW;
    private double vGaussX, vGaussY, vGaussW;

    private double mclASlow, mclAFast, mclWFast, mclWSlow; // AMCL vars

    private boolean resetParticles;

    private boolean useHeading;

    private double nParticles;

    private boolean useAdaptiveParticles;
    private double cf, dist;

    public AMCL() {
        useHeading = true;
    }

    public void init() {
        Random xrd = new Random();
        Random yrd = new Random();
        Random wrd = new Random();
        
        nParticles = Constants.FilterConstants.NUM_PARTICLES;
        for (int i = 0; i < nParticles; ++i) {
            particles.add(new Particle(
                xrd.nextDouble() * Constants.VisionConstants.Field.FIELD_WIDTH - (Constants.VisionConstants.Field.FIELD_WIDTH / 2), 
                yrd.nextDouble() * Constants.VisionConstants.Field.FIELD_HEIGHT - (Constants.VisionConstants.Field.FIELD_HEIGHT / 2), 
                wrd.nextDouble() * Math.PI * 2, 
                1 / nParticles
            ));
        }

        mGaussX = 2 / 300; // meters, 2
        mGaussW = 2 / 300; // radians, 2
        vGaussW = 5 / 300; // (degrees) radians, 5
        vGaussY = 0.1 / 300; // meters, 3
        mGaussY =  3 / 300; // meters, 3
        mclASlow = 0.01 / 300; // 0.01
        useAdaptiveParticles = false;
        mclAFast = 0.1 / 300; // 0.1
        vGaussX = 5 / 300; // meters, 5
    }

    /**
     * @param angle The particle's angle
     * @param setpoint The known angle to compare <code>angle</code> against (usually from Limelight measurements)
     * @param angleIsRadians Whether or not <code>angle</code> is in radians
     * @param setpointIsRadians Whether or not <code>setpoint</code> is in radians
     * @return
     */
    private double headingErr(double angle, double setpoint, boolean angleIsRadians, boolean setpointIsRadians) {
        if (!angleIsRadians) {
            angle = Math.toRadians(angle);
        }

        if (!setpointIsRadians) {
            setpoint = Math.toRadians(setpoint);
        }

        angle = Math.abs(angle);
        setpoint = Math.abs(setpoint);

        while (angle >= 2 * Math.PI) angle -= 2 * Math.PI;
        while (angle <= 0) angle += 2 * Math.PI;
        return frc.robot.util.Maths.Gaussian(setpoint, vGaussW, angle);
    }

    /**
     * @param angle The particle's angle in radians
     * @param setpoint The known angle to compare <code>angle</code> against in radians
     * @return
     */
    private double headingErr(double angle, double setpoint) {
        return headingErr(angle, setpoint, true, true);
    }

    /**
     * @param x Robot-relative x translation
     * @param y Robot-relative y translation
     * @param w Rotation in radians
     */
    public void updateOdometry(double x, double y, double w) {
        motionDelta.x = x;
        motionDelta.y = y;
        motionDelta.z = w;

        updateMotion();
    }

    public void setNoise(
        double mGaussX, 
        double mGaussY, 
        double mGaussW, 
        double vGaussX, 
        double vGaussY
    ) {
        this.mGaussX = mGaussX;
        this.mGaussY = mGaussY;
        this.mGaussW = mGaussW;
        this.vGaussX = vGaussX;
        this.vGaussY = vGaussY;
    }

    public void setMCLParams(
        double mclAFast,
        double mclASlow
    ) {
        this.mclAFast = mclAFast;
        this.mclASlow = mclASlow;
    }

    public void resetMCL() {
        Random xrd = new Random();
        Random yrd = new Random();
        Random wrd = new Random();
        
        nParticles = Constants.FilterConstants.NUM_PARTICLES;
        for (int i = 0; i < nParticles; ++i) {
            particles.add(new Particle(
                xrd.nextDouble() * Constants.VisionConstants.Field.FIELD_WIDTH - (Constants.VisionConstants.Field.FIELD_WIDTH / 2), 
                yrd.nextDouble() * Constants.VisionConstants.Field.FIELD_HEIGHT - (Constants.VisionConstants.Field.FIELD_HEIGHT / 2), 
                wrd.nextDouble() * Math.PI * 2, 
                1 / nParticles
            ));
        }
    }

    private void updateMotion() {
        Random xgen = new Random();
        Random ygen = new Random();
        Random wgen = new Random();

        double dx = motionDelta.x;
        double dy = motionDelta.y;
        double dw = motionDelta.z;
        for (var p : particles) {
            p.x += dx + xgen.nextGaussian(0, mGaussX);
            p.y += dy + ygen.nextGaussian(0, mGaussY);
            p.w += dw + wgen.nextGaussian(0, mGaussW);

            while (p.w >= Math.PI * 2) p.w -= Math.PI * 2;
            while (p.w < 0) p.w += Math.PI * 2;

            if (p.x > Constants.VisionConstants.Field.FIELD_WIDTH / 2) p.x = Constants.VisionConstants.Field.FIELD_WIDTH / 2;
            else if (p.x < 0 - Constants.VisionConstants.Field.FIELD_WIDTH / 2) p.x = 0 - Constants.VisionConstants.Field.FIELD_WIDTH / 2;

            if (p.y > Constants.VisionConstants.Field.FIELD_HEIGHT / 2) p.y = Constants.VisionConstants.Field.FIELD_HEIGHT / 2;
            else if (p.y < 0 - Constants.VisionConstants.Field.FIELD_HEIGHT / 2) p.y = 0 - Constants.VisionConstants.Field.FIELD_HEIGHT / 2;
        }
    }

    public void tagScanning(boolean hasTargets, int id, double[] dists, double[] campose) {
        TagDistance[] distances = new TagDistance[8];

        assert dists.length == 8;

        for (int i = 0; i < dists.length; ++i) {
            distances[i] = new TagDistance(
                Constants.VisionConstants.Field.TAGS[i].x, 
                Constants.VisionConstants.Field.TAGS[i].y, 
                dists[i]
            );
        }

        tagDistances = new ArrayList<>(Arrays.asList(distances));

        updatePerceptionPoints(hasTargets, id, campose);
    }

    private void updatePerceptionPoints(boolean hasTargets, int id, double[] campose) {
        int numPoints = tagDistances.size();
        double sumWeight = 0;
        double wAvg = 0;

        for (var p : particles) {
            double prob = 1;
            double cmpsProb = 0;

            if (numPoints > 0) {
                resetParticles = false;

                for (var d : tagDistances) {
                    if (d.distance > 0) {
                        double tagDist = d.distance + Maths.normalDistribution(0, Math.hypot(vGaussX, vGaussY));
                        double particleDistance = Math.hypot(p.x - d.x, p.y - d.y);
                        double distanceDiff = Math.abs(particleDistance - tagDist);
                        prob *= frc.robot.util.Maths.Gaussian(0, Math.hypot(vGaussX, vGaussY), distanceDiff);
                    } else {
                        prob *= 1;
                    }
                }

                p.weight = prob;

                if (useHeading && hasTargets) {
                    cmpsProb = 1 / headingErr(p.w, 
                        Math.toRadians((campose[2] + Constants.VisionConstants.Field.TAGS[id - 1].z) % 360)  +
                            Maths.normalDistribution(0, vGaussW)
                    );
                    p.weight *= cmpsProb;
                }
                sumWeight += p.weight;
            } else {
                resetParticles = true;
            }
        }

        for (var p : particles) {
            wAvg += p.weight / nParticles;
            p.weight /= sumWeight;
        }

        mclWSlow += mclASlow * (wAvg - mclWSlow);
        mclWFast += mclAFast * (wAvg - mclWFast);

        try {
            lowVarResampling();
        } catch (Exception e) {
            System.out.println("Oh noes.");
        }
    }

    private void lowVarResampling() {
        ArrayList<Particle> newParticles = new ArrayList<>();
        Random random = new Random();
        double resetProb = Math.max(0, 1 - (mclWFast / mclWSlow));
        double r = random.nextDouble() * (1 / nParticles);
        double c = particles.get(0).weight;
        bestEstimate = particles.get(0);
        int id = 0;

        double meanX = 0;
        double meanY = 0;
        double sin = 0, cos = 0;
        double orient = 0;

        for (int j = 0; j < Constants.FilterConstants.NUM_PARTICLES; ++j) {
            double rand = Math.random();
            if (rand < resetProb || resetParticles) {
                resetParticles = false;
                newParticles.add(new Particle(
                    Math.random() * Constants.VisionConstants.Field.FIELD_WIDTH - (Constants.VisionConstants.Field.FIELD_WIDTH / 2), 
                    Math.random() * Constants.VisionConstants.Field.FIELD_HEIGHT - (Constants.VisionConstants.Field.FIELD_HEIGHT / 2), 
                    Math.random() * Math.PI * 2, 
                    1 / Constants.FilterConstants.NUM_PARTICLES
                ));
            } else {
                double U = r + ((double) j / nParticles);
                while (U > c) {
                    id += 1;
                    c += particles.get(id).weight;
                }
                if (particles.get(id).weight > bestEstimate.weight) {
                    bestEstimate = particles.get(id).clone();
                }
                newParticles.add(particles.get(id).clone());
            }
        }

        particles = newParticles;

        for (var p : particles) {
            meanX += p.x;
            meanY += p.y;
            sin += Math.sin(p.w);
            cos += Math.cos(p.w);
            p.weight = 1 / Constants.FilterConstants.NUM_PARTICLES;
        }

        meanX /= Constants.FilterConstants.NUM_PARTICLES;
        meanY /= Constants.FilterConstants.NUM_PARTICLES;
        orient = Math.atan2(sin, cos);

        meanEstimate.x = meanX;
        meanEstimate.y = meanY;
        meanEstimate.w = orient;

        if (useAdaptiveParticles) {
            cf = (dist / nParticles) / 18.38067;
            if (cf >= 0.08) nParticles = cf * Constants.FilterConstants.NUM_PARTICLES;
            else nParticles = Constants.FilterConstants.MIN_PARTICLES;
        } else {
            nParticles = Constants.FilterConstants.NUM_PARTICLES;
        }
    }

    public Particle getBestEstimate() {
        bestEstimate = particles.get(0);

        for (int j = 0; j < nParticles; ++j) {
            if (particles.get(j).weight > bestEstimate.weight) {
                bestEstimate = particles.get(j);
            }

        }
        return bestEstimate;
    }

    public Particle getAverageEstimate() {
        meanEstimate = new Particle(0, 0, 0, 0);

        double meanX = 0, meanY = 0, meanW = 0;
        for (var p : particles) {
            meanX += p.x;
            meanY += p.y;
            meanW += p.w;
        }

        meanEstimate.x = meanX / particles.size();
        meanEstimate.y = meanY / particles.size();
        meanEstimate.w = meanW / particles.size();
        meanEstimate.weight = 1 / particles.size();
        return meanEstimate;
    }

    public void outputNParticles() {
        SmartDashboard.putNumber("Number of particles", nParticles);
    }

    @Override
    public String toString() {
        String res = "";
        for (var p : particles) res += p + "\n";
        return res;
    }
}
