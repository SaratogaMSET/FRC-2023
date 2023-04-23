use rand::Rng;
use rand_distr::{Bernoulli, Distribution};
use std::f64::consts::PI;

#[inline]
pub fn gaussian(mu: f64, sigma: f64, x: f64) -> f64 {
    f64::exp(-(f64::powf(mu - x, 2.0)) / f64::powf(sigma, 2.0) / 2.0)
        / f64::sqrt(2.0 * PI * f64::powf(sigma, 2.0))
}

#[inline]
pub fn uniform_random_distribution(min: f64, max: f64) -> f64 {
    rand::thread_rng().sample::<f64, _>(rand_distr::Uniform::new(min, max))
}

#[inline]
pub fn normal_distribution(mean: f64, stddev: f64) -> f64 {
    rand::thread_rng().sample::<f64, _>(rand_distr::Normal::new(mean, stddev).unwrap())
}

#[inline]
pub fn bernoulli_distribution(p: f64) -> bool {
    Bernoulli::new(p).unwrap().sample(&mut rand::thread_rng())
}
