use crate::constants::NUM_TAGS;

#[derive(Debug)]
pub struct RobotData {
    pub odom: OdomData,
    pub vision: VisionData,
}

#[derive(Debug)]
pub struct OdomData {
    pub id: i32,
    pub x: f64,
    pub y: f64,
    pub w: f64,
}

#[derive(Debug)]
pub struct VisionData {
    pub id: i32,
    pub has_targets: bool,
    pub tag_id: i32,
    pub distances: [f64; NUM_TAGS],
    pub campose: [f64; 3],
}
