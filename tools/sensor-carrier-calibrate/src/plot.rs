//! SVG plots to check the fits by eye.

use std::error::Error;
use std::path::Path;

use plotters::coord::types::RangedCoordf64;
use plotters::prelude::*;

use crate::calibrate::{
    Calibration, GyroFit, MAG_NT_PER_LSB, MagFit, imu_board_dps, mag_board_counts,
};
use crate::fit::Estimate;
use crate::log::{Log, Mag};

type Chart<'a> = ChartContext<'a, SVGBackend<'a>, Cartesian2d<RangedCoordf64, RangedCoordf64>>;

const SIZE: (u32, u32) = (1200, 500);
const COLORS: [RGBColor; 3] = [
    RGBColor(0x2a, 0x78, 0xd6),
    RGBColor(0xeb, 0x68, 0x34),
    RGBColor(0x1b, 0xaf, 0x7a),
];
// Still samples are averaged over this long so the noise does not hide drift.
const STILL_MEAN_S: f64 = 0.1;

/// Writes `gyro_<IMU>.svg`, `mag_<MAG>.svg` and `latency_<sensor>.svg` into `dir`.
pub fn write(
    dir: &Path,
    log: &Log,
    calibration: &Calibration,
    estimates: &[Estimate],
) -> Result<(), Box<dyn Error>> {
    std::fs::create_dir_all(dir)?;
    for gyro in &calibration.gyro {
        gyro_still(&dir.join(format!("gyro_{}.svg", gyro.sensor)), gyro)?;
    }
    for (samples, mag) in log.mag.iter().zip(&calibration.mag) {
        mag_magnitude(&dir.join(format!("mag_{}.svg", mag.sensor)), samples, mag)?;
    }
    for estimate in estimates {
        latency_profile(
            &dir.join(format!("latency_{}.svg", estimate.name)),
            estimate,
        )?;
    }
    Ok(())
}

/// Board-frame rate per axis while still, with the fitted bias as a line.
fn gyro_still(path: &Path, gyro: &GyroFit) -> Result<(), Box<dyn Error>> {
    let means: Vec<(f64, [f64; 3])> = gyro
        .still
        .chunk_by(|a, b| (a.t / STILL_MEAN_S).floor() == (b.t / STILL_MEAN_S).floor())
        .map(|window| {
            let n = window.len() as f64;
            let t = window.iter().map(|s| s.t).sum::<f64>() / n;
            let rate = std::array::from_fn(|axis| {
                window
                    .iter()
                    .map(|s| imu_board_dps(s.gyro_raw)[axis])
                    .sum::<f64>()
                    / n
            });
            (t, rate)
        })
        .collect();

    let root = SVGBackend::new(path, SIZE).into_drawing_area();
    let mut chart = chart(
        &root,
        &format!(
            "{} gyro rate while still, lines at the fitted bias",
            gyro.sensor
        ),
        means.iter().map(|(t, _)| *t),
        means.iter().flat_map(|(_, rate)| *rate),
        "time (s)",
        "rate (dps)",
    )?;
    let (t_min, t_max) = (chart.x_range().start, chart.x_range().end);
    for (axis, name) in ["x", "y", "z"].into_iter().enumerate() {
        let color = COLORS[axis];
        chart
            .draw_series(
                means
                    .iter()
                    .map(|(t, rate)| Circle::new((*t, rate[axis]), 2, color.filled())),
            )?
            .label(name)
            .legend(move |(x, y)| Circle::new((x + 10, y), 4, color.filled()));
        if let Some(bias) = gyro.bias_dps {
            chart.draw_series(LineSeries::new(
                [(t_min, bias[axis]), (t_max, bias[axis])],
                color.stroke_width(2),
            ))?;
        }
    }
    finish(&root, &mut chart)
}

/// Field magnitude before and after the correction. A good fit makes the
/// corrected magnitude flat.
fn mag_magnitude(path: &Path, samples: &[Mag], mag: &MagFit) -> Result<(), Box<dyn Error>> {
    let ut = |v: [f64; 3]| v.iter().map(|c| c * c).sum::<f64>().sqrt() * MAG_NT_PER_LSB / 1000.0;
    let board = |s: &Mag| mag_board_counts(s.field_raw);
    let uncorrected: Vec<(f64, f64)> = samples
        .iter()
        .map(|s| (s.t, ut(board(s).map(f64::from))))
        .collect();
    let corrected: Option<Vec<(f64, f64)>> = mag.fit.map(|fit| {
        samples
            .iter()
            .map(|s| (s.t, ut(fit.apply(board(s)).map(f64::from))))
            .collect()
    });

    let root = SVGBackend::new(path, SIZE).into_drawing_area();
    let mut chart = chart(
        &root,
        &format!("{} field magnitude", mag.sensor),
        uncorrected.iter().map(|(t, _)| *t),
        uncorrected
            .iter()
            .chain(corrected.iter().flatten())
            .map(|(_, v)| *v),
        "time (s)",
        "field (uT)",
    )?;
    let corrected = corrected.map(|points| ("corrected", points));
    draw_lines(
        &mut chart,
        std::iter::once(("uncorrected", uncorrected)).chain(corrected),
    )?;
    finish(&root, &mut chart)
}

/// How much worse the fit gets as the latency moves away from the estimate,
/// against what the covariance predicts. A clear valley means the log
/// determines the latency.
fn latency_profile(path: &Path, estimate: &Estimate) -> Result<(), Box<dyn Error>> {
    let point = |latency_s: f64, value: f64| (latency_s * 1e3, value);
    let fit: Vec<(f64, f64)> = estimate
        .profile
        .iter()
        .map(|p| point(p.latency_s, p.increase))
        .collect();
    let predicted: Vec<(f64, f64)> = estimate
        .profile
        .iter()
        .map(|p| point(p.latency_s, p.predicted))
        .collect();

    let root = SVGBackend::new(path, SIZE).into_drawing_area();
    let mut chart = chart(
        &root,
        &format!("{} latency fit", estimate.name),
        fit.iter().map(|(t, _)| *t),
        fit.iter().chain(&predicted).map(|(_, v)| *v),
        "latency (ms)",
        "fit worsening (χ² increase)",
    )?;
    draw_lines(&mut chart, [("fit", fit), ("predicted", predicted)])?;
    finish(&root, &mut chart)
}

/// A titled chart whose axes span the given values.
fn chart<'a>(
    root: &DrawingArea<SVGBackend<'a>, plotters::coord::Shift>,
    title: &str,
    xs: impl Iterator<Item = f64>,
    ys: impl Iterator<Item = f64>,
    x_desc: &str,
    y_desc: &str,
) -> Result<Chart<'a>, Box<dyn Error>> {
    root.fill(&WHITE)?;
    let (x_min, x_max) = range(xs);
    let (y_min, y_max) = range(ys);
    let pad = 0.1 * (y_max - y_min);
    let mut chart = ChartBuilder::on(root)
        .caption(title, ("sans-serif", 22))
        .margin(15)
        .x_label_area_size(40)
        .y_label_area_size(70)
        .build_cartesian_2d(x_min..x_max, (y_min - pad)..(y_max + pad))?;
    chart
        .configure_mesh()
        .x_desc(x_desc)
        .y_desc(y_desc)
        .y_label_formatter(&tick)
        .draw()?;
    Ok(chart)
}

fn draw_lines<'a>(
    chart: &mut Chart,
    series: impl IntoIterator<Item = (&'a str, Vec<(f64, f64)>)>,
) -> Result<(), Box<dyn Error>> {
    for ((name, points), color) in series.into_iter().zip(COLORS) {
        chart
            .draw_series(LineSeries::new(points, color.stroke_width(2)))?
            .label(name)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 20, y)], color.stroke_width(2)));
    }
    Ok(())
}

fn finish(
    root: &DrawingArea<SVGBackend, plotters::coord::Shift>,
    chart: &mut Chart,
) -> Result<(), Box<dyn Error>> {
    chart
        .configure_series_labels()
        .background_style(WHITE.mix(0.8))
        .border_style(BLACK)
        .draw()?;
    root.present()?;
    Ok(())
}

/// Short axis labels: scientific notation for large values.
fn tick(value: &f64) -> String {
    if value.abs() >= 1e4 {
        format!("{value:.1e}")
    } else {
        format!("{}", (value * 1e4).round() / 1e4)
    }
}

/// Smallest and largest value, widened if they are equal or missing.
fn range(values: impl Iterator<Item = f64>) -> (f64, f64) {
    let (min, max) = values.fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), v| {
        (min.min(v), max.max(v))
    });
    if min > max {
        (0.0, 1.0)
    } else if min == max {
        (min - 0.5, max + 0.5)
    } else {
        (min, max)
    }
}
