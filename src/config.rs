// Runtime configuration — Phase 7.1.
//
// Loaded from `config.toml` (or the path given to `Config::load`).
// Falls back to sensible Orion hardware defaults if the file is absent.
//
// File format: simple key = value lines (comments with #, section headers [ignored]).

#[derive(Debug, Clone)]
pub struct Config {
    // ── Network ──────────────────────────────────────────
    /// UDP port to listen on (default 8008).
    pub port: u16,

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
}

impl Default for Config {
    fn default() -> Self {
        Self {
            port: 8008,
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
                "port" => {
                    if let Ok(v) = val.parse::<u16>() { cfg.port = v; }
                }
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
