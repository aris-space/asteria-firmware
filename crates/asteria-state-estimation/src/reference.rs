//! Conversion between geodetic coordinates and the local north-east-down frame.

const WGS84_SEMI_MAJOR_AXIS_M: f64 = 6_378_137.0;
const WGS84_ECCENTRICITY_SQUARED: f64 = 6.694_379_990_14e-3;
const DEG_TO_RAD: f64 = core::f64::consts::PI / 180.0;

/// Fixed origin of the north-east-down frame that estimator inputs and outputs use.
///
/// North and east are arc lengths along the WGS84 ellipsoid, using its meridian and
/// prime-vertical radii at the reference latitude. Down is the negated MSL height difference, so
/// it has no curvature error at any distance. The horizontal scale error is below one centimetre
/// per kilometre within a few kilometres of the reference.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeodeticReference {
    latitude_rad: f64,
    longitude_rad: f64,
    height_msl_m: f32,
    north_m_per_rad: f64,
    east_m_per_rad: f64,
}

impl GeodeticReference {
    /// Creates a reference at a latitude, longitude, and MSL height.
    ///
    /// Returns `None` for non-finite values or a latitude outside `(-90°, 90°)`.
    #[must_use]
    pub fn new(latitude_deg: f64, longitude_deg: f64, height_msl_m: f32) -> Option<Self> {
        if !latitude_deg.is_finite()
            || !longitude_deg.is_finite()
            || !height_msl_m.is_finite()
            || latitude_deg.abs() >= 90.0
        {
            return None;
        }
        let latitude_rad = latitude_deg * DEG_TO_RAD;
        let sin_latitude = libm::sin(latitude_rad);
        let curvature = 1.0 - WGS84_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude;
        let meridian_radius_m = WGS84_SEMI_MAJOR_AXIS_M * (1.0 - WGS84_ECCENTRICITY_SQUARED)
            / (curvature * libm::sqrt(curvature));
        let prime_vertical_radius_m = WGS84_SEMI_MAJOR_AXIS_M / libm::sqrt(curvature);
        Some(Self {
            latitude_rad,
            longitude_rad: longitude_deg * DEG_TO_RAD,
            height_msl_m,
            north_m_per_rad: meridian_radius_m,
            east_m_per_rad: prime_vertical_radius_m * libm::cos(latitude_rad),
        })
    }

    /// Converts a geodetic position into north, east, and down metres from the reference.
    #[must_use]
    #[allow(clippy::cast_possible_truncation)]
    pub fn project(&self, position: GeodeticPosition) -> [f32; 3] {
        let north_m =
            (position.latitude_deg * DEG_TO_RAD - self.latitude_rad) * self.north_m_per_rad;
        let east_m =
            (position.longitude_deg * DEG_TO_RAD - self.longitude_rad) * self.east_m_per_rad;
        [
            north_m as f32,
            east_m as f32,
            self.height_msl_m - position.height_msl_m,
        ]
    }

    /// Converts north, east, and down metres from the reference into a geodetic position.
    ///
    /// This is the exact inverse of [`Self::project`].
    #[must_use]
    pub fn unproject(&self, position_ned_m: [f32; 3]) -> GeodeticPosition {
        let [north_m, east_m, down_m] = position_ned_m;
        GeodeticPosition {
            latitude_deg: (self.latitude_rad + f64::from(north_m) / self.north_m_per_rad)
                / DEG_TO_RAD,
            longitude_deg: (self.longitude_rad + f64::from(east_m) / self.east_m_per_rad)
                / DEG_TO_RAD,
            height_msl_m: self.height_msl_m - down_m,
        }
    }
}

/// A WGS84 latitude and longitude with a height above mean sea level.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeodeticPosition {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub height_msl_m: f32,
}

#[cfg(test)]
mod tests {
    use super::{GeodeticPosition, GeodeticReference};

    const REFERENCE: GeodeticPosition = GeodeticPosition {
        latitude_deg: 47.0,
        longitude_deg: 8.0,
        height_msl_m: 500.0,
    };

    fn reference() -> GeodeticReference {
        GeodeticReference::new(
            REFERENCE.latitude_deg,
            REFERENCE.longitude_deg,
            REFERENCE.height_msl_m,
        )
        .unwrap()
    }

    #[test]
    fn reference_projects_to_origin() {
        assert_eq!(reference().project(REFERENCE), [0.0, 0.0, 0.0]);
    }

    #[test]
    fn unproject_inverts_project() {
        let reference = reference();
        let position = GeodeticPosition {
            latitude_deg: 47.03,
            longitude_deg: 8.04,
            height_msl_m: 3_500.0,
        };
        let round_trip = reference.unproject(reference.project(position));
        // f32 metres limit the round trip to millimetres, about 1e-8 degrees.
        assert!((round_trip.latitude_deg - position.latitude_deg).abs() < 1e-7);
        assert!((round_trip.longitude_deg - position.longitude_deg).abs() < 1e-7);
        assert!((round_trip.height_msl_m - position.height_msl_m).abs() < 1e-3);
    }

    #[test]
    fn ellipsoid_radii_give_known_distances() {
        // One arc minute of latitude at 47° is 1852.85 m on WGS84, and one of longitude is
        // 1267.60 m, from the standard series for degree lengths.
        let reference = reference();
        let [north_m, _, _] = reference.project(GeodeticPosition {
            latitude_deg: 47.0 + 1.0 / 60.0,
            ..REFERENCE
        });
        let [_, east_m, _] = reference.project(GeodeticPosition {
            longitude_deg: 8.0 + 1.0 / 60.0,
            ..REFERENCE
        });
        assert!((north_m - 1_852.85).abs() < 0.05, "{north_m}");
        assert!((east_m - 1_267.60).abs() < 0.05, "{east_m}");
    }

    #[test]
    fn down_is_negated_height_difference() {
        let [_, _, down_m] = reference().project(GeodeticPosition {
            height_msl_m: 1_500.0,
            ..REFERENCE
        });
        assert_eq!(down_m, -1_000.0);
    }

    #[test]
    fn rejects_poles_and_non_finite_values() {
        assert!(GeodeticReference::new(90.0, 0.0, 0.0).is_none());
        assert!(GeodeticReference::new(f64::NAN, 0.0, 0.0).is_none());
        assert!(GeodeticReference::new(0.0, 0.0, f32::INFINITY).is_none());
    }
}
