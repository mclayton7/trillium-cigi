// Platform position and attitude state.
//
// Loaded from config at startup; a future UDP source will update this in real-time
// via a tokio::sync::watch channel (see the TBD src/platform_udp.rs).

use crate::config::Config;

/// Current platform position and attitude.
#[derive(Debug, Clone)]
pub struct PlatformState {
    /// Geodetic latitude (rad).
    pub lat_rad: f64,
    /// Geodetic longitude (rad).
    pub lon_rad: f64,
    /// Altitude above ellipsoid (m).
    pub alt_m: f64,
    /// Roll angle (rad, right-wing-down positive).
    pub roll_rad: f32,
    /// Pitch angle (rad, nose-up positive).
    pub pitch_rad: f32,
    /// Yaw / heading (rad, clockwise-from-north positive).
    pub yaw_rad: f32,
}

impl PlatformState {
    /// Construct from runtime config (static source).
    pub fn from_config(cfg: &Config) -> Self {
        Self {
            lat_rad: cfg.platform_lat,
            lon_rad: cfg.platform_lon,
            alt_m: cfg.platform_alt,
            roll_rad: cfg.platform_roll,
            pitch_rad: cfg.platform_pitch,
            yaw_rad: cfg.platform_yaw,
        }
    }
}
