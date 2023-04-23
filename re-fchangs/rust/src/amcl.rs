use std::{collections::HashMap, f64::consts::PI};

use crate::{
    constants::*, math_util::*, particle::Particle, point3::Point3, tag_distance::TagDistance,
};

pub struct Amcl {
    distances: HashMap<i32, TagDistance>,

    motion_delta: Point3,

    particles: Vec<Particle>,
    best_estimate: Particle,
    mean_estimate: Particle,
    weighted_average: Particle,

    motion_gauss_x: f64,
    motion_gauss_y: f64,
    motion_gauss_w: f64,

    vision_gauss_x: f64,
    vision_gauss_y: f64,
    vision_gauss_w: f64,

    mcl_aslow: f64,
    mcl_afast: f64,
    mcl_wslow: f64,
    mcl_wfast: f64,

    use_heading: bool,
}

impl Amcl {
    pub fn new() -> Amcl {
        Amcl {
            distances: HashMap::new(),
            motion_delta: Point3(0.0, 0.0, 0.0),
            particles: Vec::new(),
            best_estimate: Particle::default(),
            mean_estimate: Particle::default(),
            weighted_average: Particle::default(),
            motion_gauss_x: 0.1,
            motion_gauss_y: 0.1,
            motion_gauss_w: 0.15,
            vision_gauss_x: 0.1,
            vision_gauss_y: 0.1,
            vision_gauss_w: 0.15,
            mcl_aslow: 0.01,
            mcl_afast: 0.1,
            mcl_wslow: 0.0,
            mcl_wfast: 0.0,
            use_heading: true,
        }
    }

    pub fn init(&mut self) {
        for _ in 0..NUM_PARTICES {
            self.particles.push(Particle(
                uniform_random_distribution(-FIELD_WIDTH_OFFSET, FIELD_WIDTH - FIELD_WIDTH_OFFSET),
                uniform_random_distribution(
                    -FIELD_HEIGHT_OFFSET,
                    FIELD_HEIGHT - FIELD_HEIGHT_OFFSET,
                ),
                uniform_random_distribution(0.0, PI * 2.0),
                1.0 / NUM_PARTICES as f64,
            ));
        }
    }

    fn heading_err<const ANGLE_IS_RADIANS: bool, const SETPOINT_IS_RADIANS: bool>(
        mut angle: f64,
        mut setpoint: f64,
        sigma: f64,
    ) -> f64 {
        if !ANGLE_IS_RADIANS {
            angle = angle.to_radians();
        }

        if !SETPOINT_IS_RADIANS {
            setpoint = setpoint.to_radians();
        }

        angle = angle.abs();
        setpoint = setpoint.abs();

        angle %= 2.0 * PI;
        while angle < 0.0 {
            angle += 2.0 * PI;
        }
        gaussian(setpoint, sigma, angle)
    }

    pub fn update_odometry(&mut self, x: f64, y: f64, w: f64) {
        self.motion_delta.0 = x;
        self.motion_delta.1 = y;
        self.motion_delta.2 = w;
        self.update_motion();
    }

    fn update_motion(&mut self) {
        let dx = self.motion_delta.x();
        let dy = self.motion_delta.y();
        let dw = self.motion_delta.z();

        for mut p in &mut self.particles {
            p.0 += dx + normal_distribution(0.0, self.motion_gauss_x);
            p.1 += dy + normal_distribution(0.0, self.motion_gauss_y);
            p.2 += dw + normal_distribution(0.0, self.motion_gauss_w);

            p.2 %= 2.0 * PI;
            if p.2 < 0.0 {
                p.2 += 2.0 * PI;
            }

            if p.0 > FIELD_WIDTH - FIELD_WIDTH_OFFSET {
                p.0 = FIELD_WIDTH - FIELD_WIDTH_OFFSET;
            } else if p.0 < -FIELD_WIDTH_OFFSET {
                p.0 = -FIELD_WIDTH_OFFSET;
            }

            if p.1 > FIELD_HEIGHT - FIELD_HEIGHT_OFFSET {
                p.1 = FIELD_HEIGHT - FIELD_HEIGHT_OFFSET;
            } else if p.1 < -FIELD_HEIGHT_OFFSET {
                p.1 = -FIELD_HEIGHT_OFFSET;
            }
        }
    }

    pub fn tag_scanning(&mut self, id: i32, dists: [f64; NUM_TAGS], campose: [f64; 3]) {
        self.distances.clear();
        for i in 0..dists.len() {
            if dists[i] > 0.0 {
                self.distances.insert(
                    i as i32 + 1,
                    TagDistance(TAG_ARRAY[i].x(), TAG_ARRAY[i].y(), dists[i]),
                );
            }
        }

        self.update_perception(id, campose);
    }

    fn update_perception(&mut self, id: i32, campose: [f64; 3]) {
        let num_points = self.distances.len();
        let mut sum_weight: f64 = 0.0;
        let mut w_avg: f64 = 0.0;

        for p in &mut self.particles {
            let mut prob: f64 = 1.0;

            if num_points > 0 {
                for d in &mut self.distances.values_mut() {
                    let tag_dist: f64 = d.distance()
                        + normal_distribution(
                            0.0,
                            f64::hypot(self.vision_gauss_x, self.vision_gauss_y),
                        );
                    let particle_distance: f64 = f64::hypot(p.x() - d.x(), p.y() - d.y());
                    let distance_diff: f64 = f64::abs(particle_distance - tag_dist);
                    prob *= gaussian(
                        0.0,
                        f64::hypot(self.vision_gauss_x, self.vision_gauss_y),
                        distance_diff,
                    );
                }

                p.3 = prob;

                if self.use_heading && id > 1 {
                    let cmps_prob = Self::heading_err::<true, true>(
                        p.w(),
                        (campose[2] + TAG_ARRAY[id as usize - 1_usize].z()).to_radians()
                            % (2.0 * PI)
                            + normal_distribution(0.0, self.vision_gauss_w),
                        self.vision_gauss_w,
                    );

                    p.3 *= cmps_prob;
                }

                sum_weight += p.weight();
            }
        }

        for p in &mut self.particles {
            w_avg += p.weight() / NUM_PARTICES as f64;
            p.3 /= sum_weight;
        }

        self.mcl_wslow += self.mcl_aslow * (w_avg - self.mcl_wslow);
        self.mcl_wfast += self.mcl_afast * (w_avg - self.mcl_wfast);

        self.compute_weighted_average();
        self.low_var_resampling();
    }

    fn low_var_resampling(&mut self) {
        let reset_prob: f64 = f64::clamp(1.0 - (self.mcl_wfast - self.mcl_wslow), 0.0, 1.0);
        let r = uniform_random_distribution(0.0, 1.0 / NUM_PARTICES as f64);
        let mut new_particles: Vec<Particle> = Vec::new();
        let mut c = self.particles.first().unwrap().weight();
        let mut id = 0;
        self.best_estimate = *self.particles.first().unwrap();

        for j in 0..NUM_PARTICES {
            if bernoulli_distribution(reset_prob) {
                new_particles.push(Particle(
                    uniform_random_distribution(
                        -FIELD_WIDTH_OFFSET,
                        FIELD_WIDTH - FIELD_WIDTH_OFFSET,
                    ),
                    uniform_random_distribution(
                        -FIELD_HEIGHT_OFFSET,
                        FIELD_HEIGHT - FIELD_HEIGHT_OFFSET,
                    ),
                    uniform_random_distribution(0.0, PI * 2.0),
                    1.0 / NUM_PARTICES as f64,
                ));
            } else {
                let u = r + (j as f64 / NUM_PARTICES as f64);
                while u > c {
                    id += 1;
                    c += self.particles.get(id).unwrap().weight();
                }

                if self.particles.get(id).unwrap().weight() > self.best_estimate.weight() {
                    self.best_estimate = *self.particles.get(id).unwrap();
                }

                new_particles.push(*self.particles.get(id).unwrap());
            }
        }

        self.particles = new_particles;

        // Recompute best and average estimates after resampling
        self.get_best_estimate();
        self.get_average_estimate();
    }

    pub fn get_best_estimate(&mut self) -> Particle {
        self.best_estimate = *self.particles.first().unwrap();

        for i in 0..NUM_PARTICES {
            if self.particles.get(i).unwrap().weight() > self.best_estimate.weight() {
                self.best_estimate = *self.particles.get(i).unwrap();
            }
        }

        self.best_estimate
    }

    pub fn get_average_estimate(&mut self) -> Particle {
        self.mean_estimate = Particle::default();

        let (mut mean_x, mut mean_y, mut mean_w) = (0.0, 0.0, 0.0);
        for p in &mut self.particles {
            mean_x += p.x();
            mean_y += p.y();
            mean_w += p.w();
        }

        self.mean_estimate.0 = mean_x / NUM_PARTICES as f64;
        self.mean_estimate.1 = mean_y / NUM_PARTICES as f64;
        self.mean_estimate.2 = mean_w / NUM_PARTICES as f64;

        self.mean_estimate
    }

    pub fn compute_weighted_average(&mut self) -> Particle {
        let (mut mean_x, mut mean_y, mut mean_w) = (0.0, 0.0, 0.0);
        for p in &mut self.particles {
            mean_x += p.x() * p.weight();
            mean_y += p.y() * p.weight();
            mean_w += p.w() * p.weight();
        }

        self.weighted_average = Particle(mean_x, mean_y, mean_w, 0.0);
        self.weighted_average
    }

    pub fn get_weighted_average(&mut self) -> Particle {
        self.weighted_average
    }
}

impl Default for Amcl {
    fn default() -> Self {
        Self::new()
    }
}