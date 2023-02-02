package testclient.filter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import org.opencv.core.Point3;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import testclient.Constants;
import testclient.filterOLD.Maths;
import testclient.wrappers.TagDistance;

// FIXME check if EVERYTHING (besides LL campose/botpose) is in radians
public class AMCL {
    private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    private ArrayList<TagDistance> tagDistances = new ArrayList<>();

    private Point3 robotPose; // original robot pose
    private Point3 motionDelta; // robot frame motion odometry <-- maybe not?

    private Particle[] particles;
    private Particle bestEstimate;
    private Particle meanEstimate;

    private double mGaussX, mGaussY, mGaussW;
    private double vGaussX, vGaussY;

    private double mclFieldptsVar, mclHeadingVar; // vision and heading variance
    private double mclASlow, mclAFast, mclWFast, mclWSlow; // AMCL vars

    private double cf, nParticles; // adaptive particles vars
    private boolean resetParticles;
    private boolean useAdaptiveParticles;

    private boolean useHeading;

    public AMCL() {
        useHeading = true;
    }

    public void init() {
        robotPose = new Point3(0, 0, 0);

        cf = 0;

        Random xrd = new Random();
        Random yrd = new Random();
        Random wrd = new Random();
        
        nParticles = Constants.FilterConstants.NUM_PARTICLES;
        particles = new Particle[(int) nParticles];
        for (Particle p : particles) {
            p.x = xrd.nextDouble() * Constants.VisionConstants.Field.FIELD_WIDTH;
            p.y = yrd.nextDouble() * Constants.VisionConstants.Field.FIELD_HEIGHT;
            p.w = wrd.nextDouble() * Math.PI * 2;
            p.weight = 1 / nParticles;
        }

        mclFieldptsVar = 0.3;
        mGaussX = 2;
        mGaussW = 2;
        mclHeadingVar = 0.323;
        vGaussY = 3;
        mGaussY =  3;
        mclASlow = 0.01;
        useAdaptiveParticles = true;
        mclAFast = 0.1;
        vGaussX = 5;
    }

    /**
     * DO NOT USE THIS METHOD!!!!!!
     * @param angle
     * @param radians Whether or not angle is in radians
     * @return Particle's heading error w.r.t. average particle's heading
     */
    public double headingErr(double angle, boolean radians) {
        // FIXME use "botpose_targetspace" LL NT key as setpoint? absolute values of particle and botpose_targetspace thetas
        // to compute deltas instead of using robotPose because using "known robot position" is BS and stupid and such a
        // lazy cop-out and defeats the entire point of a particle filter????????????
        if (radians) {
            while (angle >= 2 * Math.PI) angle -= 2 * Math.PI;
            while (angle <= 0) angle += 2 * Math.PI;
            return Math.exp(Math.abs(angle - robotPose.z) / 2 * mclHeadingVar * mclHeadingVar);
        } else {
            while (angle >= 360) angle -= 360;
            while (angle <= 0) angle += 360;
            double res = Math.toRadians(angle);
            return Math.exp(Math.abs(res - robotPose.z) / 2 * mclHeadingVar * mclHeadingVar);
        }
    }

    /**
     * @param angle The particle's angle
     * @param setpoint The known angle to compare <code>angle</code> against (usually from Limelight measurements)
     * @param angleIsRadians Whether or not <code>angle</code> is in radians
     * @param setpointIsRadians Whether or not <code>setpoint</code> is in radians
     * @return
     */
    public double headingErr(double angle, double setpoint, boolean angleIsRadians, boolean setpointIsRadians) {
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

    public void updateOdometry(double x, double y, double w) {
        motionDelta.x = x;
        motionDelta.y = y;
        motionDelta.z = w;

        updateMotion();
    }

    public void updatePose(double x, double y, double w) {
        robotPose.x = x;
        robotPose.y = y;
        robotPose.z = w;
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
        for (Particle p : particles) {
            p.x = xrd.nextDouble() * Constants.VisionConstants.Field.FIELD_WIDTH;
            p.y = yrd.nextDouble() * Constants.VisionConstants.Field.FIELD_HEIGHT;
            p.w = wrd.nextDouble() * Math.PI * 2;
            p.weight = 1 / nParticles;
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
        }
    }

    public void tagScanning(double[] dists) {
        TagDistance[] distances = new TagDistance[8];

        assert dists.length == 8;

        for (int i = 0; i < dists.length; ++i) {
            distances[i] = new TagDistance(
                Constants.VisionConstants.Field.TAGS[i].x, 
                Constants.VisionConstants.Field.TAGS[i].y, 
                dists[i]
            );
        }

        tagDistances = (ArrayList<TagDistance>) Arrays.asList(distances);

        updatePerceptionPoints();
    }

    private void updatePerceptionPoints() {
        int numPoints = tagDistances.size();
        double sumWeight = 0;
        double wAvg = 0;

        for (var p : particles) {
            double prob = 1;
            double cmpsProb = 0;

            if (numPoints > 0) {
                resetParticles = false;

                for (var d : tagDistances) {
                    double tagDist = d.distance + Maths.normalDistribution(0, Math.hypot(vGaussX, vGaussY));
                    double particleDistance = Math.hypot(p.x - d.x, p.y - d.y);
                    double distanceDiff = Math.abs(particleDistance - tagDist);
                    prob *= Math.exp((-distanceDiff * distanceDiff) / (2 * mclFieldptsVar * mclFieldptsVar));
                }

                p.weight = prob;

                if (useHeading && limelightTable.getEntry("tv").getInteger(0) == 1) {
                    cmpsProb = 1 / headingErr(p.w, 
                        limelightTable.getEntry("campose").getDoubleArray(new double[6])[4],
                        false,
                        true
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

        lowVarResampling();
    }

    private void lowVarResampling() {
        ArrayList<Particle> newParticles = new ArrayList<>();
        Random random = new Random();
        double resetProb = Math.max(0, 1 - (mclWFast / mclWSlow));
        double r = random.nextDouble() * (1 / nParticles);
        double c = particles[0].w;
        bestEstimate = particles[0];
        int id = 0;

        double meanX = 0;
        double meanY = 0;
        double sin = 0, cos = 0;
        double orient = 0;

        for (int j = 0; j < nParticles; ++j) {
            double rand = Math.random();
            if (rand < resetProb || resetParticles) {
                resetParticles = false;
                newParticles.add(new Particle(
                    Math.random() * Constants.VisionConstants.Field.FIELD_WIDTH, 
                    Math.random() * Constants.VisionConstants.Field.FIELD_HEIGHT, 
                    Math.random() * Math.PI * 2, 
                    1 / nParticles
                ));
            } else {
                double U = r + ((double) j / nParticles);
                while (U > c) {
                    id += 1;
                    c += particles[id].weight;
                }
                if (particles[id].weight > bestEstimate.weight) {
                    bestEstimate = particles[id];
                }

                newParticles.add(particles[id]);
            }
        }

        particles = (Particle[]) newParticles.toArray();

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
            cf = (dist / nParticles) / 1;
            if (cf >= 0.08) nParticles = cf * Constants.FilterConstants.NUM_PARTICLES;
            else nParticles = Constants.FilterConstants.MIN_PARTICLES;
        } else {
            nParticles = Constants.FilterConstants.NUM_PARTICLES;
        }
    }

    public Particle getBestEstimate() {
        return bestEstimate;
    }

    public Particle getAverageEstimate() {
        return meanEstimate;
    }
}
