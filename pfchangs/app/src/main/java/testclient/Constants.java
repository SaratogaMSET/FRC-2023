package testclient;

import org.opencv.core.Point3;

public class Constants {
    public static class VisionConstants {
        public static class Field {
            public static final double FIELD_WIDTH = 16.54175;
            public static final double FIELD_HEIGHT = 8.0137;
            public static final int NUM_TAGS = 8;
            public static final Point3[] TAGS = {
                new Point3(Tags.ID_1.x, Tags.ID_1.y, Tags.ID_1.z),
                new Point3(Tags.ID_2.x, Tags.ID_2.y, Tags.ID_2.z),
                new Point3(Tags.ID_3.x, Tags.ID_3.y, Tags.ID_3.z),
                new Point3(Tags.ID_4.x, Tags.ID_4.y, Tags.ID_4.z),
                new Point3(Tags.ID_5.x, Tags.ID_5.y, Tags.ID_5.z),
                new Point3(Tags.ID_6.x, Tags.ID_6.y, Tags.ID_6.z),
                new Point3(Tags.ID_7.x, Tags.ID_7.y, Tags.ID_7.z),
                new Point3(Tags.ID_8.x, Tags.ID_8.y, Tags.ID_8.z)
            };
        }

        public static enum Tags {
            // FIXME - we might have to reverse the rotations (1-4 = 180 and 4-8 = 0)
            /* ID_1(15.513558, 1.071626, 0),
            ID_2(15.513558, 2.748026, 0),
            ID_3(15.513558, 4.424426, 0),
            ID_4(16.178784, 6.749796, 0),
            ID_5(0.36195, 6.749796, 180),
            ID_6(1.02743, 4.424426, 180),
            ID_7(1.02743, 2.748026, 180),
            ID_8(1.02743, 1.071626, 180); */
            ID_1(15.513558 - Field.FIELD_WIDTH / 2, 1.071626 - Field.FIELD_HEIGHT / 2, 0),
            ID_2(15.513558 - Field.FIELD_WIDTH / 2, 2.748026 - Field.FIELD_HEIGHT / 2, 0),
            ID_3(15.513558 - Field.FIELD_WIDTH / 2, 4.424426 - Field.FIELD_HEIGHT / 2, 0),
            ID_4(16.178784 - Field.FIELD_WIDTH / 2, 6.749796 - Field.FIELD_HEIGHT / 2, 0),
            ID_5(0.36195 - Field.FIELD_WIDTH / 2, 6.749796 - Field.FIELD_HEIGHT / 2, 180),
            ID_6(1.02743 - Field.FIELD_WIDTH / 2, 4.424426 - Field.FIELD_HEIGHT / 2, 180),
            ID_7(1.02743 - Field.FIELD_WIDTH / 2, 2.748026 - Field.FIELD_HEIGHT / 2, 180),
            ID_8(1. - Field.FIELD_WIDTH / 2, 1.071626 - Field.FIELD_HEIGHT / 2, 180);

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
        public static final int MIN_PARTICLES = 200;
        public static final double FNOISE = 2;
        public static final double TNOISE = 2;
        public static final double SNOISE = 2;
    }
}
