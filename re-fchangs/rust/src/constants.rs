use crate::tag::Tag;

pub const FIELD_WIDTH: f64 = 16.54175;
pub const FIELD_HEIGHT: f64 = 8.0137;
pub const FIELD_WIDTH_OFFSET: f64 = 0.0;
pub const FIELD_HEIGHT_OFFSET: f64 = 0.0;

pub const NUM_TAGS: usize = 8;
pub const TAG_ARRAY: [Tag; NUM_TAGS] = [ID_1, ID_2, ID_3, ID_4, ID_5, ID_6, ID_7, ID_8];

pub const ID_1: Tag = Tag(
    15.513558 - FIELD_WIDTH_OFFSET,
    1.071626 - FIELD_HEIGHT_OFFSET,
    0.0,
);
pub const ID_2: Tag = Tag(
    15.513558 - FIELD_WIDTH_OFFSET,
    2.748026 - FIELD_HEIGHT_OFFSET,
    0.0,
);
pub const ID_3: Tag = Tag(
    15.513558 - FIELD_WIDTH_OFFSET,
    4.424426 - FIELD_HEIGHT_OFFSET,
    0.0,
);
pub const ID_4: Tag = Tag(
    16.178784 - FIELD_WIDTH_OFFSET,
    6.749796 - FIELD_HEIGHT_OFFSET,
    0.0,
);
pub const ID_5: Tag = Tag(
    0.36195 - FIELD_WIDTH_OFFSET,
    6.749796 - FIELD_HEIGHT_OFFSET,
    180.0,
);
pub const ID_6: Tag = Tag(
    1.02743 - FIELD_WIDTH_OFFSET,
    4.424426 - FIELD_HEIGHT_OFFSET,
    180.0,
);
pub const ID_7: Tag = Tag(
    1.02743 - FIELD_WIDTH_OFFSET,
    2.748026 - FIELD_HEIGHT_OFFSET,
    180.0,
);
pub const ID_8: Tag = Tag(
    1.02743 - FIELD_WIDTH_OFFSET,
    1.071626 - FIELD_HEIGHT_OFFSET,
    180.0,
);

pub const NUM_PARTICES: usize = 10000;
