//! SVG plots to check a calibration by eye: the gyro rate while still against
//! the fitted bias, and the magnetometer field magnitude before and after the
//! hard- and soft-iron correction.

use std::error::Error;
use std::path::Path;

use plotters::prelude::*;

use crate::calibrate::{
    Calibration, GyroFit, MAG_NT_PER_LSB, MagFit, imu_board_dps, mag_board_counts,
};
use crate::log::{Log, Mag};

const SIZE: (u32, u32) = (1200, 500);
const COLORS: [RGBColor; 3] = [
    RGBColor(0x2a, 0x78, 0xd6),
    RGBColor(0xeb, 0x68, 0x34),
    RGBColor(0x1b, 0xaf, 0x7a),
];
// Still samples are averaged over this long so the noise does not hide drift.
const STILL_MEAN_S: f64 = 0.1;

/// Writes `gyro_<IMU>.svg` and `mag_<MAG>.svg` into `dir`.
pub fn write(dir: &Path, log: &Log, calibration: &Calibration) -> Result<(), Box<dyn Error>> {
    std::fs::create_dir_all(dir)?;
    for gyro in &calibration.gyro {
        gyro_still(&dir.join(format!("gyro_{}.svg", gyro.sensor)), gyro)?;
    }
    for (samples, mag) in log.mag.iter().zip(&calibration.mag) {
        mag_magnitude(&dir.join(format!("mag_{}.svg", mag.sensor)), samples, mag)?;
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
    let (t_min, t_max) = range(means.iter().map(|(t, _)| *t));
    let (y_min, y_max) = range(means.iter().flat_map(|(_, rate)| *rate));
    let pad = 0.1 * (y_max - y_min).max(0.01);

    let root = SVGBackend::new(path, SIZE).into_drawing_area();
    root.fill(&WHITE)?;
    let title = match gyro.bias_dps {
        Some(b) => format!(
            "{} gyro while still ({STILL_MEAN_S} s means), bias {:.4}, {:.4}, {:.4} dps",
            gyro.sensor, b[0], b[1], b[2]
        ),
        None => format!("{} gyro while still: too little still data", gyro.sensor),
    };
    let mut chart = ChartBuilder::on(&root)
        .caption(title, ("sans-serif", 22))
        .margin(15)
        .x_label_area_size(40)
        .y_label_area_size(70)
        .build_cartesian_2d(t_min..t_max, (y_min - pad)..(y_max + pad))?;
    chart
        .configure_mesh()
        .x_desc("time (s)")
        .y_desc("rate (dps)")
        .draw()?;
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
    chart
        .configure_series_labels()
        .background_style(WHITE.mix(0.8))
        .border_style(BLACK)
        .draw()?;
    root.present()?;
    Ok(())
}

/// Field magnitude over the session before and after the correction. A good
/// fit makes the corrected magnitude flat at the fitted field strength.
fn mag_magnitude(path: &Path, samples: &[Mag], mag: &MagFit) -> Result<(), Box<dyn Error>> {
    let ut = |v: [f64; 3]| v.iter().map(|c| c * c).sum::<f64>().sqrt() * MAG_NT_PER_LSB / 1000.0;
    let board = |s: &Mag| mag_board_counts(s.field_raw);
    let uncorrected: Vec<(f64, f64)> = samples
        .iter()
        .map(|s| (s.t, ut(board(s).map(f64::from))))
        .collect();
    let corrected: Vec<(f64, f64)> = match &mag.fit {
        Some(fit) => samples
            .iter()
            .map(|s| (s.t, ut(fit.apply(board(s)).map(f64::from))))
            .collect(),
        None => Vec::new(),
    };
    let (t_min, t_max) = range(uncorrected.iter().map(|(t, _)| *t));
    let (y_min, y_max) = range(uncorrected.iter().chain(&corrected).map(|(_, v)| *v));
    let pad = 0.1 * (y_max - y_min).max(1.0);

    let root = SVGBackend::new(path, SIZE).into_drawing_area();
    root.fill(&WHITE)?;
    let title = match &mag.fit {
        Some(fit) => format!(
            "{} field magnitude, fitted {:.1} uT, error {:.2} %",
            mag.sensor,
            fit.field_strength as f64 * MAG_NT_PER_LSB / 1000.0,
            fit.fit_error_percent
        ),
        None => format!("{} field magnitude: no fit", mag.sensor),
    };
    let mut chart = ChartBuilder::on(&root)
        .caption(title, ("sans-serif", 22))
        .margin(15)
        .x_label_area_size(40)
        .y_label_area_size(70)
        .build_cartesian_2d(t_min..t_max, (y_min - pad)..(y_max + pad))?;
    chart
        .configure_mesh()
        .x_desc("time (s)")
        .y_desc("field (uT)")
        .draw()?;
    for (index, (name, points)) in [("uncorrected", uncorrected), ("corrected", corrected)]
        .into_iter()
        .enumerate()
    {
        let color = COLORS[index];
        chart
            .draw_series(LineSeries::new(points, color.stroke_width(2)))?
            .label(name)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 20, y)], color.stroke_width(2)));
    }
    chart
        .configure_series_labels()
        .background_style(WHITE.mix(0.8))
        .border_style(BLACK)
        .draw()?;
    root.present()?;
    Ok(())
}

/// Smallest and largest value, or `(0, 1)` if there are none.
fn range(values: impl Iterator<Item = f64>) -> (f64, f64) {
    let (min, max) = values.fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), v| {
        (min.min(v), max.max(v))
    });
    if min <= max { (min, max) } else { (0.0, 1.0) }
}
