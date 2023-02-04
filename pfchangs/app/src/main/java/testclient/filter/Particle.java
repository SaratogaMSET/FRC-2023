package testclient.filter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Particle {
    public double x;
    public double y;
    public double w;
    public double weight;

    public Particle(double x, double y, double w, double weight) {
        this.x = x;
        this.y = y;
        this.w = w;
        this.weight = weight;
    }

    public Pose2d toPose2d() {
        return new Pose2d(new Translation2d(x, y), new Rotation2d(w));
    }

    @Override
    public String toString() {
        return "[" + x + ", " + y + ", " + w + ", " + weight + "]";
    }
}
