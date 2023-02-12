package testclient.wrappers;

public class TagDistance {
    public final double x;
    public final double y;
    public double distance;
    public double distanceX;
    public double distanceY;

    public TagDistance(double x, double y, double distance, double distanceX, double distanceY) {
        this.x = x;
        this.y = y;
        this.distance = distance;
        this.distanceX = distanceX;
        this.distanceY = distanceY;
    }

    @Override
    public String toString() {
        return "[" + x + ", " + y + ", " + distance + "]";
    }
}
