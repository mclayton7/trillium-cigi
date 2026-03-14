// Platform position, attitude, and velocity state.
//
// Real-time updates are delivered via a tokio::sync::watch channel.
// The main loop holds the receiver and calls `.borrow().clone()` each tick.
// Any implementor of PlatformSource drives the sender side.

pub mod mavlink;
pub mod stanag4586;
pub mod static_source;

use crate::config::Config;
use tokio::sync::watch;

pub use mavlink::MavLinkSource;
pub use stanag4586::Stanag4586Source;
pub use static_source::StaticSource;

/// Current platform position, attitude, and velocity.
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
    /// Velocity north (m/s).
    pub vel_north_m_s: f32,
    /// Velocity east (m/s).
    pub vel_east_m_s: f32,
    /// Velocity down (m/s).
    pub vel_down_m_s: f32,
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
            ..Self::default()
        }
    }
}

impl Default for PlatformState {
    fn default() -> Self {
        Self {
            lat_rad: 0.0,
            lon_rad: 0.0,
            alt_m: 0.0,
            roll_rad: 0.0,
            pitch_rad: 0.0,
            yaw_rad: 0.0,
            vel_north_m_s: 0.0,
            vel_east_m_s: 0.0,
            vel_down_m_s: 0.0,
        }
    }
}

/// Trait for platform state sources.
///
/// Implementors drive the writer side of a watch channel.
/// The main loop holds the receiver; adding a new source means
/// creating a new file and implementing `run()`.
pub trait PlatformSource: Send + 'static {
    async fn run(self, tx: watch::Sender<PlatformState>);
}
