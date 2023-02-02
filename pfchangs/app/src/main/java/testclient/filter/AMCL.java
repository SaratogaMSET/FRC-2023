package testclient.filter;

import java.util.ArrayList;
import java.util.Random;

import org.opencv.core.Point3;

import testclient.Constants;
import testclient.wrappers.TagDistance;

// FIXME check if EVERYTHING (besides LL campose/botpose) is in radians
public class AMCL {
    private ArrayList<TagDistance> tagDistances = new ArrayList<>();

    private Point3 robotPose; // original robot pose
    private Point3 motionDelta; // robot frame motion odometry
    
    private Particle[] particles;
    private Particle bestEstimate;
    private Particle meanEstimate;

    private double xDev, yDev, wDev;
    private double mGaussX, mGaussY, mGaussW;
    private double vGaussX, vGaussY;

    private double mclFieldptsVar, mclHeadingVar; // vision and heading variance
    private double mclASlow, mclAFast, mclWFast, mclWSlow; // AMCL vars

    private double cf, nParticles; // adaptive particles vars
    private boolean resetParticles;
    private boolean useAdaptiveParticles;

    private boolean useHeading;

    private double mclPt2LineVar, mclPtAngleVar, mclLine2LineVar;

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
        mclPt2LineVar = 60;
        mGaussW = 2;
        mclHeadingVar = 0.323;
        mclPtAngleVar = 0.4;
        vGaussY = 3;
        mGaussY =  3;
        mclASlow = 0.01;
        useAdaptiveParticles = true;
        mclAFast = 0.1;
        vGaussX = 5;
        mclLine2LineVar = 0.2;
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
        double mclFieldptsVar,
        double mclPt2LineVar,
        double mclPtAngleVar,
        double mclLine2LineVar
    ) {
        this.mclAFast = mclAFast;
        this.mclASlow = mclASlow;
        this.mclHeadingVar = mclHeadingVar;
        this.useAdaptiveParticles = useAdaptiveParticles;
        this.mclFieldptsVar = mclFieldptsVar;
        this.mclPt2LineVar = mclPt2LineVar;
        this.mclPtAngleVar = mclPtAngleVar;
        this.mclLine2LineVar = mclLine2LineVar;
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

    /* 
     * Our equivalent of LineScanning():
     * Get the IDs of all visible tags
     * The end result: push_back all visible tags x, y, and distance into tagDistances
     */
}
