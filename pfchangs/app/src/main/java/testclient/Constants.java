package testclient;

import org.opencv.core.Point;

public class Constants {
    public static class VisionConstants {
        public static class Field {
            public static final double FIELD_WIDTH = 16.54175f;
            public static final double FIELD_HEIGHT = 8.0137f;
            public static final int NUM_TAGS = 8;
            public static final Point[] TAGS = {
                new Point(Tags.ID_1.x, Tags.ID_1.y),
                new Point(Tags.ID_2.x, Tags.ID_2.y),
                new Point(Tags.ID_3.x, Tags.ID_3.y),
                new Point(Tags.ID_4.x, Tags.ID_4.y),
                new Point(Tags.ID_5.x, Tags.ID_5.y),
                new Point(Tags.ID_6.x, Tags.ID_6.y),
                new Point(Tags.ID_7.x, Tags.ID_7.y),
                new Point(Tags.ID_8.x, Tags.ID_8.y)
            };
        }

        public static enum Tags {
            ID_1(15.513558, 1.071626, 0.462788),
            ID_2(15.513558, 2.748026, 0.462788),
            ID_3(15.513558, 4.424426, 0.462788),
            ID_4(16.178784, 6.749796, 0.695452),
            ID_5(0.36195, 6.749796, 0.695452),
            ID_6(1.02743, 4.424426, 0.462788),
            ID_7(1.02743, 2.748026, 0.462788),
            ID_8(1.02743, 1.071626, 0.462788);

            public final double x;
            public final double y;
            public final double z;

            Tags(double x, double y, double z) {
                this.x = x;
                this.y = y;
                this.z = z;
            }
        }
    }

    public static class FilterConstants {
        public static final int NUM_PARTICLES = 2000;
        public static final int MIN_PARTICLES = 50;
        public static final double FNOISE = 2;
        public static final double TNOISE = 2;
        public static final double SNOISE = 2;
    }
}
