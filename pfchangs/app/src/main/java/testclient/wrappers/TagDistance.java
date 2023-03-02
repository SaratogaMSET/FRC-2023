package testclient.wrappers;

public class TagDistance {
    public final double x;
    public final double y;
    public double distance;

    public TagDistance(double x, double y, double distance) {
        this.x = x;
        this.y = y;
        this.distance = distance;
    }

    @Override
    public String toString() {
        return "[" + x + ", " + y + ", " + distance + "]";
    }
}
