use criterion::{criterion_group, criterion_main, Criterion};

fn mcl_bench_odom(c: &mut Criterion) {
    let mut pf = rust::amcl::Amcl::default();
    pf.init();

    c.bench_function("MCL Odometry Update", |b| {
        b.iter(|| pf.update_odometry(0.0, 0.0, 0.0))
    });
}

fn mcl_bench_resample(c: &mut Criterion) {
    let mut pf = rust::amcl::Amcl::default();
    pf.init();
    pf.update_odometry(0.0, 0.0, 0.0);

    c.bench_function("MCL Resampling", |b| {
        b.iter(|| {
            pf.tag_scanning(
                1,
                [2.5, 2.5, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0],
                [0.0, 0.0, 0.0],
            )
        })
    });
}

fn mcl_bench_average(c: &mut Criterion) {
    let mut pf = rust::amcl::Amcl::default();
    pf.init();
    pf.update_odometry(0.0, 0.0, 0.0);
    pf.tag_scanning(
        1,
        [2.5, 2.5, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0],
        [0.0, 0.0, 0.0],
    );

    c.bench_function("MCL Weighted Average", |b| {
        b.iter(|| pf.compute_weighted_average())
    });
}

criterion_group!(benches, mcl_bench_odom, mcl_bench_resample, mcl_bench_average);
criterion_main!(benches);
