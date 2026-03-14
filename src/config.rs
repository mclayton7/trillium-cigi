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

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    fn write_temp(name: &str, content: &str) -> String {
        let mut p = std::env::temp_dir();
        p.push(format!("cigi_trillium_cfg_test_{}.toml", name));
        fs::write(&p, content).expect("write temp config");
        p.to_string_lossy().into_owned()
    }

    // ── Config::load ─────────────────────────────────────────────────────────

    #[test]
    fn load_missing_file_returns_default() {
        let cfg = Config::load("/tmp/cigi_trillium_no_such_file_xyz999.toml");
        let def = Config::default();
        assert_eq!(cfg.orion_listen_port, def.orion_listen_port);
        assert_eq!(cfg.scene_generator_ip, def.scene_generator_ip);
        assert_eq!(cfg.max_slew_rate, def.max_slew_rate);
        assert_eq!(cfg.platform_lat, def.platform_lat);
    }

    #[test]
    fn load_known_keys_parsed_correctly() {
        let content = concat!(
            "max_slew_rate_deg_s = 120.0\n",
            "orion_listen_port = 9090\n",
            "platform_lat_deg = 45.0\n",
            "mavlink_system_id = 7\n",
            "stanag_vehicle_id = 42\n",
        );
        let path = write_temp("known_keys", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        assert!((cfg.max_slew_rate - 120_f32.to_radians()).abs() < 1e-5);
        assert_eq!(cfg.orion_listen_port, 9090);
        assert!((cfg.platform_lat - 45_f64.to_radians()).abs() < 1e-10);
        assert_eq!(cfg.mavlink_system_id, 7);
        assert_eq!(cfg.stanag_vehicle_id, 42);
    }

    #[test]
    fn load_comments_and_section_headers_ignored() {
        let content = concat!(
            "# this is a comment\n",
            "[network]\n",
            "orion_listen_port = 1234\n",
        );
        let path = write_temp("comments", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        assert_eq!(cfg.orion_listen_port, 1234);
        assert_eq!(cfg.scene_generator_ip, "127.0.0.1");
    }

    #[test]
    fn load_unknown_keys_silently_ignored() {
        let content = concat!(
            "totally_unknown_key = 999\n",
            "orion_listen_port = 5555\n",
        );
        let path = write_temp("unknown_keys", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        assert_eq!(cfg.orion_listen_port, 5555);
        assert_eq!(cfg.cigi_listen_port, 8101); // default untouched
    }

    #[test]
    fn load_whitespace_around_equals_handled() {
        let content = concat!("orion_listen_port=7777\n", "cigi_listen_port = 6666\n");
        let path = write_temp("whitespace_eq", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        assert_eq!(cfg.orion_listen_port, 7777);
        assert_eq!(cfg.cigi_listen_port, 6666);
    }

    #[test]
    fn load_quoted_string_values_stripped() {
        let content = "scene_generator_ip = \"192.168.1.50\"\n";
        let path = write_temp("quoted_string", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        assert_eq!(cfg.scene_generator_ip, "192.168.1.50");
    }

    #[test]
    fn load_platform_lat_deg_to_radians() {
        let content = "platform_lat_deg = 90.0\n";
        let path = write_temp("platform_lat", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        assert!((cfg.platform_lat - std::f64::consts::FRAC_PI_2).abs() < 1e-10);
    }

    #[test]
    fn load_mavlink_system_id_max() {
        let content = "mavlink_system_id = 255\n";
        let path = write_temp("mavlink_id", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        assert_eq!(cfg.mavlink_system_id, 255_u8);
    }

    #[test]
    fn load_stanag_vehicle_id_negative() {
        let content = "stanag_vehicle_id = -1\n";
        let path = write_temp("stanag_vid", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        assert_eq!(cfg.stanag_vehicle_id, -1_i32);
    }

    // ── Config::is_continuous_pan ─────────────────────────────────────────────

    #[test]
    fn is_continuous_pan_default_is_false() {
        assert!(!Config::default().is_continuous_pan());
    }

    #[test]
    fn is_continuous_pan_true_at_360_deg() {
        let content = "pan_limit_deg = 360.0\n";
        let path = write_temp("continuous_pan", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        assert!(cfg.is_continuous_pan());
    }

    #[test]
    fn is_continuous_pan_true_at_exact_threshold() {
        let mut cfg = Config::default();
        cfg.pan_limit = std::f32::consts::TAU - 0.001;
        assert!(cfg.is_continuous_pan());
    }

    #[test]
    fn is_continuous_pan_false_just_below_threshold() {
        let mut cfg = Config::default();
        cfg.pan_limit = std::f32::consts::TAU - 0.002;
        assert!(!cfg.is_continuous_pan());
    }

    // ── Config::fov_at_zoom ───────────────────────────────────────────────────

    #[test]
    fn fov_at_zoom_zero_returns_wide() {
        let cfg = Config::default();
        let (h, v) = cfg.fov_at_zoom(0.0);
        assert!((h - cfg.hfov_wide).abs() < 1e-6);
        assert!((v - cfg.vfov_wide).abs() < 1e-6);
    }

    #[test]
    fn fov_at_zoom_one_returns_narrow() {
        let cfg = Config::default();
        let (h, v) = cfg.fov_at_zoom(1.0);
        assert!((h - cfg.hfov_narrow).abs() < 1e-6);
        assert!((v - cfg.vfov_narrow).abs() < 1e-6);
    }

    #[test]
    fn fov_at_zoom_half_is_midpoint() {
        let cfg = Config::default();
        let (h, v) = cfg.fov_at_zoom(0.5);
        assert!((h - (cfg.hfov_wide + cfg.hfov_narrow) / 2.0).abs() < 1e-6);
        assert!((v - (cfg.vfov_wide + cfg.vfov_narrow) / 2.0).abs() < 1e-6);
    }

    #[test]
    fn fov_at_zoom_clamped_below_zero() {
        let cfg = Config::default();
        assert_eq!(cfg.fov_at_zoom(-0.1), cfg.fov_at_zoom(0.0));
    }

    #[test]
    fn fov_at_zoom_clamped_above_one() {
        let cfg = Config::default();
        assert_eq!(cfg.fov_at_zoom(1.1), cfg.fov_at_zoom(1.0));
    }

    // ── camera_fov ────────────────────────────────────────────────────────────

    #[test]
    fn camera_fov_index_0() {
        assert_eq!(camera_fov(0), (30.0, 22.5));
    }

    #[test]
    fn camera_fov_index_1() {
        assert_eq!(camera_fov(1), (5.0, 3.75));
    }

    #[test]
    fn camera_fov_index_2() {
        assert_eq!(camera_fov(2), (20.0, 15.0));
    }

    #[test]
    fn camera_fov_out_of_range_falls_back_to_cam0() {
        assert_eq!(camera_fov(99), CAMERA_TABLE[0]);
    }
}
