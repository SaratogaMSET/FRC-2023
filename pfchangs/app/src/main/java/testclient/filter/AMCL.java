package testclient.filter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import org.opencv.core.Point3;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import testclient.Constants;
import testclient.Maths;
import testclient.wrappers.TagDistance;

// FIXME check if EVERYTHING (besides LL campose/botpose) is in radians
public class AMCL {
    private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    private ArrayList<TagDistance> tagDistances = new ArrayList<>();

    private Point3 motionDelta = new Point3(); // robot frame motion odometry <-- maybe not?

    private Particle[] particles;
    // private ArrayList<Particle> particles;
    private Particle bestEstimate = new Particle(0, 0, 0, 0);
    private Particle meanEstimate = new Particle(0, 0, 0, 0);

    private double mGaussX, mGaussY, mGaussW;
    private double vGaussX, vGaussY, vGaussW;

    private double mclFieldptsVar, mclHeadingVar; // vision and heading variance
    private double mclASlow, mclAFast, mclWFast, mclWSlow; // AMCL vars

    private double cf, nParticles; // adaptive particles vars
    private boolean resetParticles;
    private boolean useAdaptiveParticles;

    private boolean useHeading;

    public AMCL() {
        useHeading = false;
    }

    public void init() {
        cf = 0;

        Random xrd = new Random();
        Random yrd = new Random();
        Random wrd = new Random();
        
        nParticles = Constants.FilterConstants.NUM_PARTICLES;
        particles = new Particle[(int) nParticles];
        for (int i = 0; i < nParticles; ++i) {
            particles[i] = new Particle(0, 0, 0, 0);
            particles[i].x = xrd.nextDouble() * Constants.VisionConstants.Field.FIELD_WIDTH - (Constants.VisionConstants.Field.FIELD_WIDTH / 2);
            particles[i].y = yrd.nextDouble() * Constants.VisionConstants.Field.FIELD_HEIGHT - (Constants.VisionConstants.Field.FIELD_HEIGHT / 2);
            particles[i].w = wrd.nextDouble() * Math.PI * 2;
            particles[i].weight = 1 / nParticles;
            // System.out.println(particles[i]);
            // System.out.println(particles[i].weight);
        }

        mclFieldptsVar = 0.3; // ?, 0.3
        mGaussX = 0.15; // meters, 2
        mGaussW = 0.25; // radians, 2
        mclHeadingVar = 0.0323; // ?, 0.323
        vGaussW = 10; // degrees, 5
        vGaussY = 0.1; // meters, 3
        mGaussY =  0.15; // meters, 3
        mclASlow = 0.01; // 0.01
        useAdaptiveParticles = false;
        mclAFast = 0.1; // 0.1
        vGaussX = 0.1; // meters, 5
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
        return Math.exp(Math.abs(angle - setpoint) / 2 * mclHeadingVar * mclHeadingVar);
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
        double mclASlow,
        double mclHeadingVar,
        boolean useAdaptiveParticles,
        double mclFieldptsVar
    ) {
        this.mclAFast = mclAFast;
        this.mclASlow = mclASlow;
        this.mclHeadingVar = mclHeadingVar;
        this.useAdaptiveParticles = useAdaptiveParticles;
        this.mclFieldptsVar = mclFieldptsVar;
    }

    public void resetMCL() {
        Random xrd = new Random();
        Random yrd = new Random();
        Random wrd = new Random();
        
        nParticles = Constants.FilterConstants.NUM_PARTICLES;
        particles = new Particle[(int) nParticles];
        for (int i = 0; i < nParticles; ++i) {
            particles[i] = new Particle(0, 0, 0, 0);
            particles[i].x = xrd.nextDouble() * Constants.VisionConstants.Field.FIELD_WIDTH - (Constants.VisionConstants.Field.FIELD_WIDTH / 2);
            particles[i].y = yrd.nextDouble() * Constants.VisionConstants.Field.FIELD_HEIGHT - (Constants.VisionConstants.Field.FIELD_HEIGHT / 2);
            particles[i].w = wrd.nextDouble() * Math.PI * 2;
            particles[i].weight = 1 / nParticles;
        }
    }

    private void updateMotion() {
        Random xgen = new Random();
        Random ygen = new Random();
        Random wgen = new Random();

        double dx = motionDelta.x;
        double dy = motionDelta.y;
        double dw = motionDelta.z;
        double c, s;
        for (var p : particles) {
            c = Math.cos(p.w);
            s = Math.sin(p.w);

            p.x += c * dx - s * dy + xgen.nextGaussian() * mGaussX;
            p.y += s * dx + c * dy + ygen.nextGaussian() * mGaussY;
            p.w += dw + wgen.nextGaussian() * mGaussW;

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
                        prob *= Math.exp((-distanceDiff * distanceDiff) / (2 * mclFieldptsVar * mclFieldptsVar));
                        // System.out.println("Before: " + p.weight);
                        // p.weight = prob;
                        // System.out.println(p.x + " " + p.y);
                        // System.out.println("Tag: " + d.x + " " + d.y);
                        // System.out.println(tagDist + " " + particleDistance + " " + distanceDiff + " " + prob + " " + p.weight);
                        // System.out.println("Prob calc: " + p.weight + " ");
                    } else {
                        prob *= 1;
                        // p.weight *= prob;
                        // System.out.println("Prob calc: " + p.weight + " ");
                    }
                }

                p.weight = prob;
                // System.out.println("Prob calc: " + p.weight + " ");

                /* if (useHeading && limelightTable.getEntry("tv").getInteger(0) == 1) {
                    cmpsProb = 1 / headingErr(p.w, 
                        Math.toRadians((campose[2] + Constants.VisionConstants.Field.TAGS[id - 1].z) % 360)  +
                            Maths.normalDistribution(0, vGaussW),
                        true,
                        true
                    );
                    p.weight *= cmpsProb;
                } */
                sumWeight += p.weight;
            } else {
                resetParticles = true;
            }
        }

        for (var p : particles) {
            wAvg += p.weight / nParticles;
            // System.out.println("Before normalized: " + p.weight);
            p.weight /= sumWeight;
            // System.out.println("normalized: " + p.weight);
        }
        // System.out.println("sum weight real: " + sumWeight);

        mclWSlow += mclASlow * (wAvg - mclWSlow);
        mclWFast += mclAFast * (wAvg - mclWFast);

        lowVarResampling();
    }

    private void lowVarResampling() {
        // System.out.println("Starting resampling.");
        ArrayList<Particle> newParticles = new ArrayList<>();
        Random random = new Random();
        double resetProb = Math.max(0, 1 - (mclWFast / mclWSlow));
        double r = random.nextDouble() * (1 / nParticles);
        double c = particles[0].weight;
        bestEstimate = particles[0];
        int id = 0;

        double meanX = 0;
        double meanY = 0;
        double sin = 0, cos = 0;
        double orient = 0;

        for (int j = 0; j < nParticles; ++j) {
            double rand = Math.random();
            if (rand < resetProb || resetParticles) {
                // System.out.println("Reset particle.");
                resetParticles = false;
                newParticles.add(new Particle(
                    Math.random() * Constants.VisionConstants.Field.FIELD_WIDTH - (Constants.VisionConstants.Field.FIELD_WIDTH / 2), 
                    Math.random() * Constants.VisionConstants.Field.FIELD_HEIGHT - (Constants.VisionConstants.Field.FIELD_HEIGHT / 2), 
                    Math.random() * Math.PI * 2, 
                    1 / nParticles
                ));
            } else {
                double U = r + ((double) j / nParticles);
                /* while (U > c) { // METHOD 1
                    id += 1;
                    // if (id >= 1999) System.out.println(id);
                    // System.out.println(particles[id].weight);
                    c += particles[id].weight;
                    // System.out.println("U: " + U + " c: " + c);
                } */
                while (U > c) { // METHOD 2
                    // System.out.println(id);
                    id += 1;
                    /* if (id >= nParticles) {
                        // U = r + ((double) j / nParticles);
                        // c = particles[0].weight;
                        id = 0;
                    } */
                    c += particles[id].weight;
                    // System.out.println(U + " " + c + " " + id + " " + j);
                }
                /* if (id >= nParticles) {
                    // U = r + ((double) j / nParticles);
                    // c = particles[0].weight;
                    id = 0;
                } */
                /* for (int i = 0; i < nParticles; ++i) { // METHOD 3
                    c += particles[i].weight;
                    if (U <= c) {
                        id = i;
                        break;
                    }
                } */
                // System.out.println("Loop finished. " + j);
                if (particles[id].weight > bestEstimate.weight) {
                    bestEstimate = particles[id];
                }
                // System.out.println("Best: " + bestEstimate);

                System.out.println("BEFORE" + j);
                System.out.println("particles[id] before best estimate: " + particles[id] + " " + id);
                // if (id - 1 > 0) System.out.println("particles[id - 1] before best estimate: " + particles[id - 1] + " " + (id - 1));
                // if (id < 19) System.out.println("particles[id + 1] before best estimate: " + particles[id + 1] + " " + (id + 1));
                newParticles.add(particles[id]);
                System.out.println("AFTER");
                System.out.println("particles[id] after best estimate: " + particles[id] + " " + id);
                // if (id - 1 > 0) System.out.println("particles[id - 1] after best estimate: " + particles[id - 1] + " " + (id - 1));
                // if (id < 19) System.out.println("particles[id + 1] after best estimate: " + particles[id + 1] + " " + (id + 1));
            }
        }

        // System.out.println("BEFORE ASSIGNMENT: \n" + this);
        // particles = newParticles.toArray(new Particle[20]);
        // System.out.println("AFTER ASSIGNMENT: \n" + this);

        double dist = 0;
        double xBest = bestEstimate.x;
        double yBest = bestEstimate.y;
        cf = 0;

        for (var p : particles) {
            meanX += p.x;
            meanY += p.y;
            sin += Math.sin(p.w);
            cos += Math.cos(p.w);
            p.weight = 1 / nParticles;

            double x = xBest - p.x;
            double y = yBest - p.y;
            dist += Math.hypot(x, y);
        }

        meanX /= nParticles;
        meanY /= nParticles;
        orient = Math.atan2(sin, cos);

        meanEstimate.x = meanX;
        meanEstimate.y = meanY;
        meanEstimate.w = orient;

        if (useAdaptiveParticles) {
            cf = (dist / nParticles) / 18.38067;
            if (cf >= 0.0004) nParticles = cf * Constants.FilterConstants.NUM_PARTICLES;
            else nParticles = Constants.FilterConstants.MIN_PARTICLES;
        } else {
            nParticles = Constants.FilterConstants.NUM_PARTICLES;
        }
    }

    public Particle getBestEstimate() {
        bestEstimate = particles[0];

        for (int j = 0; j < nParticles; ++j) {
            if (particles[j].weight > bestEstimate.weight) {
                bestEstimate = particles[j];
            }

        }
        // System.out.println(bestEstimate);
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

        meanEstimate.x = meanX / particles.length;
        meanEstimate.y = meanY / particles.length;
        meanEstimate.w = meanW / particles.length;
        meanEstimate.weight = 1 / particles.length;
        System.out.println("Average particle: " + meanEstimate);
        // System.out.println(this);
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
