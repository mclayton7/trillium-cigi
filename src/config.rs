// Runtime configuration — Phase 7.1.
//
// Loaded from `config.toml` (or the path given to `Config::load`).
// Falls back to sensible Orion hardware defaults if the file is absent.
//
// File format: simple key = value lines (comments with #, section headers [ignored]).

#[derive(Debug, Clone)]
pub struct Config {
    // ── Network ──────────────────────────────────────────
    /// TCP port to accept Trillium/Orion connections (default 8008).
    pub orion_listen_port: u16,
    /// IP address of the CIGI scene generator (default "127.0.0.1").
    pub scene_generator_ip: String,
    /// UDP port of the CIGI scene generator (default 8100).
    pub scene_generator_cigi_port: u16,
    /// UDP port to receive CIGI responses from the scene generator (default 8101).
    pub cigi_listen_port: u16,

    // ── Platform position/attitude ───────────────────────
    /// Platform latitude (rad).
    pub platform_lat: f64,
    /// Platform longitude (rad).
    pub platform_lon: f64,
    /// Platform altitude (m).
    pub platform_alt: f64,
    /// Platform roll (rad).
    pub platform_roll: f32,
    /// Platform pitch (rad).
    pub platform_pitch: f32,
    /// Platform yaw (rad).
    pub platform_yaw: f32,

    // ── Kinematics ───────────────────────────────────────
    /// Maximum slew rate (rad/s). Default: 60 °/s.
    pub max_slew_rate: f32,
    /// Maximum acceleration/deceleration (rad/s²). Default: 300 °/s².
    pub max_accel: f32,
    /// Symmetric pan limit (rad). Default: ±170°.
    pub pan_limit: f32,
    /// Tilt minimum (rad, most negative = nose-down). Default: −110°.
    pub tilt_min: f32,
    /// Tilt maximum (rad). Default: +30°.
    pub tilt_max: f32,

    // ── Camera ───────────────────────────────────────────
    /// Wide-angle horizontal FOV (rad). Default: 30°.
    pub hfov_wide: f32,
    /// Wide-angle vertical FOV (rad). Default: 22.5°.
    pub vfov_wide: f32,
    /// Narrow (zoom) horizontal FOV (rad). Default: 3°.
    pub hfov_narrow: f32,
    /// Narrow (zoom) vertical FOV (rad). Default: 2.25°.
    pub vfov_narrow: f32,

    // ── Vibration ────────────────────────────────────────
    /// Sinusoidal jitter frequency (Hz). Default: 10 Hz.
    pub jitter_freq: f32,
    /// Peak jitter amplitude (rad). Default: ~0.05° ≈ 0.87 mrad.
    pub jitter_amplitude: f32,
    /// White-noise floor (rad RMS). Default: ~0.01° ≈ 0.175 mrad.
    pub noise_floor: f32,

    // ── Platform source ───────────────────────────────
    /// Platform data source: "static" (default), "mavlink", or "stanag4586".
    pub platform_source: String,
    /// UDP port to listen for MAVLink telemetry (default 14550).
    pub mavlink_listen_port: u16,
    /// MAVLink system ID filter: 0 = accept all, 1-255 = specific vehicle.
    pub mavlink_system_id: u8,
    /// UDP port to join for STANAG 4586 multicast (default 4586).
    pub stanag_listen_port: u16,
    /// IPv4 multicast group for STANAG 4586 (default "239.0.0.1").
    pub stanag_multicast_group: String,
    /// STANAG 4586 Vehicle ID filter: 0 = accept all.
    pub stanag_vehicle_id: i32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            orion_listen_port: 8008,
            scene_generator_ip: "127.0.0.1".to_string(),
            scene_generator_cigi_port: 8100,
            cigi_listen_port: 8101,
            platform_lat: 0.0,
            platform_lon: 0.0,
            platform_alt: 0.0,
            platform_roll: 0.0,
            platform_pitch: 0.0,
            platform_yaw: 0.0,
            max_slew_rate: 60.0_f32.to_radians(),
            max_accel: 300.0_f32.to_radians(),
            pan_limit: 170.0_f32.to_radians(),
            tilt_min: (-110.0_f32).to_radians(),
            tilt_max: 30.0_f32.to_radians(),
            hfov_wide: 30.0_f32.to_radians(),
            vfov_wide: 22.5_f32.to_radians(),
            hfov_narrow: 3.0_f32.to_radians(),
            vfov_narrow: 2.25_f32.to_radians(),
            jitter_freq: 10.0,
            jitter_amplitude: 0.05_f32.to_radians(),
            noise_floor: 0.01_f32.to_radians(),
            platform_source: "static".to_string(),
            mavlink_listen_port: 14550,
            mavlink_system_id: 0,
            stanag_listen_port: 4586,
            stanag_multicast_group: "239.0.0.1".to_string(),
            stanag_vehicle_id: 0,
        }
    }
}

impl Config {
    /// Load from `path`.  Missing file → default.  Parse errors → skip that line.
    pub fn load(path: &str) -> Self {
        let mut cfg = Config::default();
        let Ok(text) = std::fs::read_to_string(path) else {
            return cfg;
        };
        for raw in text.lines() {
            let line = raw.trim();
            if line.is_empty() || line.starts_with('#') || line.starts_with('[') {
                continue;
            }
            let Some((key, val)) = line.split_once('=') else { continue };
            let key = key.trim();
            let val = val.trim();
            match key {
                "max_slew_rate_deg_s" => {
                    if let Ok(v) = val.parse::<f32>() { cfg.max_slew_rate = v.to_radians(); }
                }
                "max_accel_deg_s2" => {
                    if let Ok(v) = val.parse::<f32>() { cfg.max_accel = v.to_radians(); }
                }
                "pan_limit_deg" => {
                    if let Ok(v) = val.parse::<f32>() { cfg.pan_limit = v.to_radians(); }
                }
                "tilt_min_deg" => {
                    if let Ok(v) = val.parse::<f32>() { cfg.tilt_min = v.to_radians(); }
                }
                "tilt_max_deg" => {
                    if let Ok(v) = val.parse::<f32>() { cfg.tilt_max = v.to_radians(); }
                }
                "hfov_wide_deg" => {
                    if let Ok(v) = val.parse::<f32>() { cfg.hfov_wide = v.to_radians(); }
                }
                "vfov_wide_deg" => {
                    if let Ok(v) = val.parse::<f32>() { cfg.vfov_wide = v.to_radians(); }
                }
                "hfov_narrow_deg" => {
                    if let Ok(v) = val.parse::<f32>() { cfg.hfov_narrow = v.to_radians(); }
                }
                "vfov_narrow_deg" => {
                    if let Ok(v) = val.parse::<f32>() { cfg.vfov_narrow = v.to_radians(); }
                }
                "jitter_freq_hz" => {
                    if let Ok(v) = val.parse::<f32>() { cfg.jitter_freq = v; }
                }
                "jitter_amplitude_deg" => {
                    if let Ok(v) = val.parse::<f32>() { cfg.jitter_amplitude = v.to_radians(); }
                }
                "noise_floor_deg" => {
                    if let Ok(v) = val.parse::<f32>() { cfg.noise_floor = v.to_radians(); }
                }
                "orion_listen_port" => {
                    if let Ok(v) = val.parse::<u16>() { cfg.orion_listen_port = v; }
                }
                "scene_generator_ip" => {
                    cfg.scene_generator_ip = val.trim_matches('"').to_string();
                }
                "scene_generator_cigi_port" => {
                    if let Ok(v) = val.parse::<u16>() { cfg.scene_generator_cigi_port = v; }
                }
                "cigi_listen_port" => {
                    if let Ok(v) = val.parse::<u16>() { cfg.cigi_listen_port = v; }
                }
                "platform_lat_deg" => {
                    if let Ok(v) = val.parse::<f64>() { cfg.platform_lat = v.to_radians(); }
                }
                "platform_lon_deg" => {
                    if let Ok(v) = val.parse::<f64>() { cfg.platform_lon = v.to_radians(); }
                }
                "platform_alt_m" => {
                    if let Ok(v) = val.parse::<f64>() { cfg.platform_alt = v; }
                }
                "platform_roll_deg" => {
                    if let Ok(v) = val.parse::<f32>() { cfg.platform_roll = v.to_radians(); }
                }
                "platform_pitch_deg" => {
                    if let Ok(v) = val.parse::<f32>() { cfg.platform_pitch = v.to_radians(); }
                }
                "platform_yaw_deg" => {
                    if let Ok(v) = val.parse::<f32>() { cfg.platform_yaw = v.to_radians(); }
                }
                "platform_source" => {
                    cfg.platform_source = val.trim_matches('"').to_string();
                }
                "mavlink_listen_port" => {
                    if let Ok(v) = val.parse::<u16>() { cfg.mavlink_listen_port = v; }
                }
                "mavlink_system_id" => {
                    if let Ok(v) = val.parse::<u8>() { cfg.mavlink_system_id = v; }
                }
                "stanag_listen_port" => {
                    if let Ok(v) = val.parse::<u16>() { cfg.stanag_listen_port = v; }
                }
                "stanag_multicast_group" => {
                    cfg.stanag_multicast_group = val.trim_matches('"').to_string();
                }
                "stanag_vehicle_id" => {
                    if let Ok(v) = val.parse::<i32>() { cfg.stanag_vehicle_id = v; }
                }
                _ => {} // unknown keys silently ignored
            }
        }
        cfg
    }

    /// Returns `true` when pan rotation is continuous (no hard stop).
    ///
    /// Triggered by `pan_limit_deg >= 360` in the config file.
    pub fn is_continuous_pan(&self) -> bool {
        self.pan_limit >= std::f32::consts::TAU - 0.001 // ≥ ~359.9°
    }

    /// HFOV/VFOV interpolated by zoom level in [0, 1] (0 = wide, 1 = narrow).
    pub fn fov_at_zoom(&self, zoom: f32) -> (f32, f32) {
        let t = zoom.clamp(0.0, 1.0);
        let hfov = self.hfov_wide + t * (self.hfov_narrow - self.hfov_wide);
        let vfov = self.vfov_wide + t * (self.vfov_narrow - self.vfov_wide);
        (hfov, vfov)
    }
}

/// Per-camera FOV table (index 0 = EO wide, 1 = EO narrow, 2 = IR).
pub const CAMERA_TABLE: &[(f32, f32)] = &[
    (30.0, 22.5),  // cam 0: EO wide
    (5.0,  3.75),  // cam 1: EO narrow
    (20.0, 15.0),  // cam 2: IR
];

/// Look up (hfov_deg, vfov_deg) for a camera index. Falls back to cam 0.
pub fn camera_fov(index: i8) -> (f32, f32) {
    CAMERA_TABLE
        .get(index.max(0) as usize)
        .copied()
        .unwrap_or(CAMERA_TABLE[0])
}
