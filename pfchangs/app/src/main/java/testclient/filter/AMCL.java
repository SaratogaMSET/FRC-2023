package testclient.filter;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;

import org.opencv.core.Point3;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import testclient.Constants;
import testclient.MathX;
import testclient.wrappers.TagDistance;

// FIXME check if EVERYTHING (besides LL campose/botpose) is in radians
public class AMCL {
    private HashMap<Integer, TagDistance> distances = new HashMap<>();

    private Point3 motionDelta = new Point3(); // robot frame motion odometry <-- maybe not?

    private ArrayList<Particle> particles = new ArrayList<Particle>();
    private Particle bestEstimate = new Particle(0, 0, 0, 0);
    private Particle meanEstimate = new Particle(0, 0, 0, 0);
    private Particle weightedAverage = new Particle();

    private double mGaussX, mGaussY, mGaussW;
    private double vGaussX, vGaussY, vGaussW;

    private double mclASlow, mclAFast, mclWFast, mclWSlow; // AMCL vars

    private boolean useHeading;

    private double nParticles;

    private boolean useAdaptiveParticles;
    private double cf, dist;

    public AMCL() {
        useHeading = true;
    }

    public void init() {
        Random r = new Random();

        nParticles = Constants.FilterConstants.NUM_PARTICLES;
        for (int i = 0; i < nParticles; ++i) {
            particles.add(new Particle(
                    r.nextDouble(-Constants.FIELD_WIDTH_OFFSET,
                            Constants.FIELD_WIDTH - Constants.FIELD_WIDTH_OFFSET),
                    r.nextDouble(-Constants.FIELD_HEIGHT_OFFSET,
                            Constants.FIELD_HEIGHT - Constants.FIELD_HEIGHT_OFFSET),
                    r.nextDouble(Math.PI * 2),
                    1 / nParticles));
        }

        mGaussX = 0.1; // meters, 0.15
        mGaussW = 0.1; // radians, 0.2
        vGaussW = 0.15; // (degrees) radians, 0.15
        vGaussY = 0.1; // meters, 0.1
        mGaussY = 0.1; // meters, 0.15
        mclASlow = 0.01; // 0.01
        useAdaptiveParticles = false;
        mclAFast = 0.1; // 0.1
        vGaussX = 0.1; // meters, 0.1
    }

    /**
     * @param angle             The particle's angle
     * @param setpoint          The known angle to compare <code>angle</code>
     *                          against (usually from Limelight measurements)
     * @param angleIsRadians    Whether or not <code>angle</code> is in radians
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

        angle %= (2 * Math.PI);
        if (angle < 0)
            angle += 2 * Math.PI;
        return MathX.Gaussian(setpoint, vGaussW, angle);
    }

    /**
     * @param angle    The particle's angle in radians
     * @param setpoint The known angle to compare <code>angle</code> against in
     *                 radians
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
            double vGaussY) {
        this.mGaussX = mGaussX;
        this.mGaussY = mGaussY;
        this.mGaussW = mGaussW;
        this.vGaussX = vGaussX;
        this.vGaussY = vGaussY;
    }

    public void setMCLParams(
            double mclAFast,
            double mclASlow) {
        this.mclAFast = mclAFast;
        this.mclASlow = mclASlow;
    }

    public void resetMCL() {
        Random r = new Random();

        nParticles = Constants.FilterConstants.NUM_PARTICLES;
        for (int i = 0; i < nParticles; ++i) {
            particles.add(new Particle(
                    r.nextDouble(-Constants.FIELD_WIDTH_OFFSET,
                            Constants.FIELD_WIDTH - Constants.FIELD_WIDTH_OFFSET),
                    r.nextDouble(-Constants.FIELD_HEIGHT_OFFSET,
                            Constants.FIELD_HEIGHT - Constants.FIELD_HEIGHT_OFFSET),
                    r.nextDouble(Math.PI * 2),
                    1 / nParticles));
        }
    }

    private void updateMotion() {
        Random r = new Random();

        double dx = motionDelta.x;
        double dy = motionDelta.y;
        double dw = motionDelta.z;
        for (var p : particles) {
            p.x += dx + r.nextGaussian(0, mGaussX);
            p.y += dy + r.nextGaussian(0, mGaussY);
            p.w += dw + r.nextGaussian(0, mGaussW);

            p.w %= (2 * Math.PI);
            if (p.w < 0)
                p.w += 2 * Math.PI;

            if (p.x > Constants.FIELD_WIDTH - Constants.FIELD_WIDTH_OFFSET)
                p.x = Constants.FIELD_WIDTH - Constants.FIELD_WIDTH_OFFSET;
            else if (p.x < 0 - Constants.FIELD_WIDTH_OFFSET)
                p.x = 0 - Constants.FIELD_WIDTH_OFFSET;

            if (p.y > Constants.FIELD_HEIGHT - Constants.FIELD_HEIGHT_OFFSET)
                p.y = Constants.FIELD_HEIGHT - Constants.FIELD_HEIGHT_OFFSET;
            else if (p.y < 0 - Constants.FIELD_HEIGHT_OFFSET)
                p.y = 0 - Constants.FIELD_HEIGHT_OFFSET;
        }
    }

    public void tagScanning(int id, double[] dists, double[] campose) {
        for (int i = 0; i < dists.length; ++i) {
            if (dists[i] > 0) {
                distances.put(i + 1,
                        new TagDistance(Constants.TAG_ARR[i].x, Constants.TAG_ARR[i].y, dists[i]));
            }
        }

        updatePerceptionPoints(id, campose);
    }

    private void updatePerceptionPoints(int id, double[] campose) {
        int numPoints = distances.size();
        double sumWeight = 0;
        double wAvg = 0;

        for (var p : particles) {
            double prob = 1;
            double cmpsProb = 0;

            if (numPoints > 0) {
                for (var d : distances.values()) {
                    double tagDist = d.distance + MathX.normalDistribution(0, Math.hypot(vGaussX, vGaussY));
                    double particleDistance = Math.hypot(p.x - d.x, p.y - d.y);
                    double distanceDiff = Math.abs(particleDistance - tagDist);
                    prob *= MathX.Gaussian(0, Math.hypot(vGaussX, vGaussY), distanceDiff);
                }

                p.weight = prob;

                if (useHeading && id > 1) {
                    cmpsProb = headingErr(p.w,
                            (campose[2] + Math.toRadians(Constants.TAG_ARR[id - 1].z)) % (2 * Math.PI) +
                                    MathX.normalDistribution(0, vGaussW));
                    p.weight *= cmpsProb;
                }
                sumWeight += p.weight;
            }
        }

        for (var p : particles) {
            wAvg += p.weight / nParticles;
            p.weight /= sumWeight;
        }

        mclWSlow += mclASlow * (wAvg - mclWSlow);
        mclWFast += mclAFast * (wAvg - mclWFast);

        computeWeightedAverage();
        lowVarResampling();
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
        double orient = 0;

        for (int j = 0; j < Constants.FilterConstants.NUM_PARTICLES; ++j) {
            SmartDashboard.putNumber("resetprob", resetProb);
            if (MathX.bernoulliDistribution(resetProb)) {
                newParticles.add(new Particle(
                        random.nextDouble(-Constants.FIELD_WIDTH_OFFSET,
                                Constants.FIELD_WIDTH - Constants.FIELD_WIDTH_OFFSET),
                        random.nextDouble(-Constants.FIELD_HEIGHT_OFFSET,
                                Constants.FIELD_HEIGHT - Constants.FIELD_HEIGHT_OFFSET),
                        random.nextDouble(Math.PI * 2),
                        1 / Constants.FilterConstants.NUM_PARTICLES));
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

        dist = 0;
        cf = 0;
        bestEstimate = getBestEstimate();
        double xBest = bestEstimate.x;
        double yBest = bestEstimate.y;

        for (var p : particles) {
            meanX += p.x;
            meanY += p.y;
            orient += p.w;
            p.weight = 1 / Constants.FilterConstants.NUM_PARTICLES;

            double tmpX = xBest - p.x;
            double tmpY = yBest - p.y;
            dist += Math.hypot(tmpX, tmpY);
        }

        meanX /= Constants.FilterConstants.NUM_PARTICLES;
        meanY /= Constants.FilterConstants.NUM_PARTICLES;
        orient /= Constants.FilterConstants.NUM_PARTICLES;

        meanEstimate.x = meanX;
        meanEstimate.y = meanY;
        meanEstimate.w = orient;

        if (useAdaptiveParticles) {
            cf = (dist / nParticles) / 18.38067;
            if (cf >= 0.08)
                nParticles = cf * Constants.FilterConstants.NUM_PARTICLES;
            else
                nParticles = Constants.FilterConstants.MIN_PARTICLES;
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

        return bestEstimate.clone();
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

        return meanEstimate.clone();
    }

    public Particle computeWeightedAverage() {
        double meanX = 0, meanY = 0, meanW = 0;
        for (var p : particles) {
            meanX += p.x * p.weight;
            meanY += p.y * p.weight;
            meanW += p.w * p.weight;
        }

        weightedAverage = new Particle(meanX, meanY, meanW, 0);
        return weightedAverage.clone();
    }

    public Particle getWeightedAverage() {
        return weightedAverage.clone();
    }

    public Particle getFilteredAverage() {
        Particle tmp = getAverageEstimate();
        LinearFilter xFilter = LinearFilter.movingAverage(10);
        LinearFilter yFilter = LinearFilter.movingAverage(10);
        LinearFilter wFilter = LinearFilter.movingAverage(10);

        return new Particle(xFilter.calculate(tmp.x), yFilter.calculate(tmp.y), wFilter.calculate(tmp.w), 0);
    }

    public Particle getFilteredWeightedAverage() {
        Particle tmp = getWeightedAverage();
        LinearFilter xFilter = LinearFilter.movingAverage(10);
        LinearFilter yFilter = LinearFilter.movingAverage(10);
        LinearFilter wFilter = LinearFilter.movingAverage(10);

        return new Particle(xFilter.calculate(tmp.x), yFilter.calculate(tmp.y), wFilter.calculate(tmp.w), 0);
    }

    public void outputNParticles() {
        SmartDashboard.putNumber("Number of particles", nParticles);
    }

    @Override
    public String toString() {
        String res = "";
        for (var p : particles)
            res += p + "\n";
        return res;
    }
}
