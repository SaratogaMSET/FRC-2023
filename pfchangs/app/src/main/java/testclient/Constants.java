package testclient;

import testclient.filter.Point;

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
            // TODO find tag coordinates
            ID_1(15.513558f, 1.071626f, 0.462788f),
            ID_2(15.513558f, 2.748026f, 0.462788f),
            ID_3(15.513558f, 4.424426f, 0.462788f),
            ID_4(16.178784f, 6.749796f, 0.695452f),
            ID_5(0.36195f, 6.749796f, 0.695452f),
            ID_6(1.02743f, 4.424426f, 0.462788f),
            ID_7(1.02743f, 2.748026f, 0.462788f),
            ID_8(1.02743f, 1.071626f, 0.462788f);

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
        public static final int NUM_PARTICLES = 5000;
        public static final double FNOISE = 2;
        public static final double TNOISE = 2;
        public static final double SNOISE = 2;
    }
}
