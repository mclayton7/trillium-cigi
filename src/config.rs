// Runtime configuration.
//
// The public `Config` struct below is **flat** — every field is reachable
// via `cfg.field_name` so the ~50 field accesses across this crate don't
// move. The TOML file on disk, however, is **sectioned**: a private
// `ConfigFile` struct tree mirrors the `[network] / [platform_source] / ...`
// layout, deserialises via `serde` + the `toml` crate, and is folded into
// `Config` by a `From` impl that only overrides fields the user actually
// specified. Missing sections fall through to `Config::default()`.
//
// Adding a new config knob:
//   1. Add a field to `Config` + `Config::default`
//   2. Add the matching `Option<T>` field to the appropriate nested
//      `ConfigFile` section struct (or add a new section)
//   3. Copy the value across in `From<ConfigFile> for Config`

use serde::Deserialize;
use sim_core::geo::GimbalMount;
use sim_core::platform::{
    MavLinkSourceConfig, PlatformSourceConfig, Stanag4586SourceConfig, StaticSourceConfig,
};

/// Top-level runtime mode for the binary.
///
/// `Bridge` is the historical behaviour: bind the CIGI UDP port, run scene-
/// generator detection, and bridge Trillium ↔ CIGI.
///
/// `Simulator` runs only the local `GimbalSimulator` and emits Trillium
/// telemetry over TCP — no CIGI bind, no SG polling, no UDP host task. Useful
/// for bring-up against Trillium clients without standing up the visualisation
/// stack and for running multiple gimbal sims on one host without port
/// collisions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeMode {
    Bridge,
    Simulator,
}

impl RuntimeMode {
    pub fn as_str(self) -> &'static str {
        match self {
            RuntimeMode::Bridge => "bridge",
            RuntimeMode::Simulator => "simulator",
        }
    }

    /// Case-insensitive parse. Returns `None` for unknown values; callers
    /// decide whether to warn and fall back to a default.
    pub fn parse(s: &str) -> Option<Self> {
        match s.trim().to_ascii_lowercase().as_str() {
            "bridge" => Some(RuntimeMode::Bridge),
            "simulator" | "sim" => Some(RuntimeMode::Simulator),
            _ => None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Config {
    // ── Runtime ──────────────────────────────────────────
    /// Bridge (default) or Simulator mode. See [`RuntimeMode`].
    pub runtime_mode: RuntimeMode,

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
    /// Wide-angle HFOV (rad) — synced from `camera_table[0].0` at load.
    pub hfov_wide: f32,
    /// Wide-angle VFOV (rad).
    pub vfov_wide: f32,
    /// Narrow HFOV (rad).
    pub hfov_narrow: f32,
    /// Narrow VFOV (rad).
    pub vfov_narrow: f32,

    /// Per-camera FOV table: `[(hfov_wide_deg, vfov_wide_deg, hfov_narrow_deg, vfov_narrow_deg); 3]`.
    /// Index 0 = EO wide, 1 = EO narrow, 2 = IR. Stored in **degrees**.
    pub camera_table: [(f32, f32, f32, f32); 3],

    // ── Vibration ────────────────────────────────────────
    /// Sinusoidal jitter frequency (Hz). Default: 10 Hz.
    pub jitter_freq: f32,
    /// Peak jitter amplitude (rad). Default: ~0.05° ≈ 0.87 mrad.
    pub jitter_amplitude: f32,
    /// White-noise floor (rad RMS). Default: ~0.01° ≈ 0.175 mrad.
    pub noise_floor: f32,
    /// Second structural resonance frequency (Hz). Default: 47 Hz (non-harmonic
    /// with the 10 Hz primary to produce realistic beat patterns).
    pub jitter_freq_2: f32,
    /// Peak amplitude of the second resonance (rad). Default: ~0.01°.
    pub jitter_amplitude_2: f32,

    // ── Stabilization ─────────────────────────────────────
    /// Stabilization quality factor: 0.0 = perfect inertial stabilization,
    /// 1.0 = fully body-mounted. Default: 0.0 (Orion is inertially stabilized).
    pub stabilization_quality: f32,

    // ── Gimbal mount ──────────────────────────────────────
    /// Physical installation of the gimbal on the airframe.
    pub gimbal_mount: GimbalMount,

    // ── Atmospheric ──────────────────────────────────────────
    /// Enable atmospheric refraction correction for LOS ray-cast. Default: true.
    pub refraction_enabled: bool,

    // ── Terrain ─────────────────────────────────────────────
    /// Uniform terrain elevation above WGS84 ellipsoid (metres). Default: 0.0.
    pub terrain_elevation_m: f64,

    // ── Geopoint ──────────────────────────────────────────
    /// Default target altitude (metres) for geopoint mode. Default: 0.0.
    pub geopoint_alt_m: f64,

    // ── Track mode ──────────────────────────────────────────
    /// Proportional gain for track-mode controller. Default: 3.0.
    pub track_p_gain: f32,
    /// Derivative gain for track-mode controller. Default: 0.5.
    pub track_d_gain: f32,
    /// Track loss threshold (fractional FOV, 0–1). Default: 0.45.
    pub track_loss_threshold: f32,
    /// Angular extent of the tracking gate (deg). Default: 1.0 deg.
    /// Combined with the current HFOV to compute gate_x/y_size in pixels.
    pub track_gate_size_deg: f32,
    /// Assumed linear size of the tracked target (metres). Default: 2.0 m.
    /// Used together with slant range to derive `PrimaryTrackData::size`.
    pub track_target_size_m: f32,

    // ── Laser rangefinder ─────────────────────────────────
    /// Maximum laser rangefinder range (metres). Default: 20 km.
    /// Beyond this, `range_source` falls back to `RangeSrcNone`.
    pub laser_max_range_m: f64,

    // ── Cross-axis coupling ───────────────────────────────
    /// Cross-axis gyroscopic coupling factor. 0.0 disables the model.
    /// Non-zero values (e.g. 0.02) model imperfect direct-drive compensation.
    pub gyro_coupling_factor: f32,

    // ── Platform source ───────────────────────────────
    /// Platform source selector: `"static"` (default), `"mavlink"`, or `"stanag4586"`.
    /// Packaged into a [`PlatformSourceConfig`] by [`Config::platform_source_config`].
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
            runtime_mode: RuntimeMode::Bridge,
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
            camera_table: [
                (30.0, 22.5, 3.0, 2.25),   // cam 0: EO wide
                (5.0, 3.75, 0.5, 0.375),    // cam 1: EO narrow
                (20.0, 15.0, 2.0, 1.5),     // cam 2: IR
            ],
            jitter_freq: 10.0,
            jitter_amplitude: 0.05_f32.to_radians(),
            noise_floor: 0.01_f32.to_radians(),
            jitter_freq_2: 47.0,
            jitter_amplitude_2: 0.01_f32.to_radians(),
            stabilization_quality: 0.0,
            gimbal_mount: GimbalMount::default(),
            refraction_enabled: true,
            terrain_elevation_m: 0.0,
            geopoint_alt_m: 0.0,
            track_p_gain: 3.0,
            track_d_gain: 0.5,
            track_loss_threshold: 0.45,
            track_gate_size_deg: 1.0,
            track_target_size_m: 2.0,
            laser_max_range_m: 20_000.0,
            gyro_coupling_factor: 0.0,
            platform_source: "static".to_string(),
            mavlink_listen_port: 14550,
            mavlink_system_id: 0,
            stanag_listen_port: 4586,
            stanag_multicast_group: "239.0.0.1".to_string(),
            stanag_vehicle_id: 0,
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════
//  TOML file format (private)
// ═════════════════════════════════════════════════════════════════════════
//
// Every field is `Option<T>` so that missing keys fall through to the
// `Config::default()` values in the `From<ConfigFile> for Config` impl.

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct ConfigFile {
    runtime: RuntimeSection,
    network: NetworkSection,
    platform_source: PlatformSourceSection,
    kinematics: KinematicsSection,
    camera: CameraSection,
    vibration: VibrationSection,
    stabilization: StabilizationSection,
    gimbal_mount: GimbalMountSection,
    atmosphere: AtmosphereSection,
    terrain: TerrainSection,
    geopoint: GeopointSection,
    track: TrackSection,
    laser: LaserSection,
    coupling: CouplingSection,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct RuntimeSection {
    /// `"bridge"` (default) | `"simulator"`. See [`RuntimeMode`].
    mode: Option<String>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct NetworkSection {
    orion_listen_port: Option<u16>,
    scene_generator_ip: Option<String>,
    scene_generator_cigi_port: Option<u16>,
    cigi_listen_port: Option<u16>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct PlatformSourceSection {
    /// `"static"` | `"mavlink"` | `"stanag4586"`
    kind: Option<String>,
    mavlink: PlatformMavLinkSection,
    stanag4586: PlatformStanagSection,
    #[serde(rename = "static")]
    static_: PlatformStaticSection,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct PlatformMavLinkSection {
    listen_port: Option<u16>,
    system_id: Option<u8>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct PlatformStanagSection {
    listen_port: Option<u16>,
    multicast_group: Option<String>,
    vehicle_id_filter: Option<i32>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct PlatformStaticSection {
    lat_deg: Option<f64>,
    lon_deg: Option<f64>,
    alt_m: Option<f64>,
    roll_deg: Option<f32>,
    pitch_deg: Option<f32>,
    yaw_deg: Option<f32>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct KinematicsSection {
    max_slew_rate_deg_s: Option<f32>,
    max_accel_deg_s2: Option<f32>,
    pan_limit_deg: Option<f32>,
    tilt_min_deg: Option<f32>,
    tilt_max_deg: Option<f32>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct CameraSection {
    eo_wide: CameraEntrySection,
    eo_narrow: CameraEntrySection,
    ir: CameraEntrySection,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct CameraEntrySection {
    hfov_wide_deg: Option<f32>,
    vfov_wide_deg: Option<f32>,
    hfov_narrow_deg: Option<f32>,
    vfov_narrow_deg: Option<f32>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct VibrationSection {
    jitter_freq_hz: Option<f32>,
    jitter_amplitude_deg: Option<f32>,
    noise_floor_deg: Option<f32>,
    jitter_freq_2_hz: Option<f32>,
    jitter_amplitude_2_deg: Option<f32>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct StabilizationSection {
    quality: Option<f32>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct GimbalMountSection {
    fwd_m: Option<f64>,
    right_m: Option<f64>,
    down_m: Option<f64>,
    roll_deg: Option<f64>,
    pitch_deg: Option<f64>,
    yaw_deg: Option<f64>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct AtmosphereSection {
    refraction_enabled: Option<bool>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct TerrainSection {
    elevation_m: Option<f64>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct GeopointSection {
    alt_m: Option<f64>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct TrackSection {
    p_gain: Option<f32>,
    d_gain: Option<f32>,
    loss_threshold: Option<f32>,
    gate_size_deg: Option<f32>,
    target_size_m: Option<f32>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct LaserSection {
    max_range_m: Option<f64>,
}

#[derive(Debug, Default, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct CouplingSection {
    gyro_factor: Option<f32>,
}

impl From<ConfigFile> for Config {
    fn from(f: ConfigFile) -> Self {
        let mut cfg = Config::default();

        // ── Runtime ──
        if let Some(mode) = f.runtime.mode {
            match RuntimeMode::parse(&mode) {
                Some(m) => cfg.runtime_mode = m,
                None => eprintln!(
                    "[config] unknown runtime.mode {mode:?} — defaulting to bridge"
                ),
            }
        }

        // ── Network ──
        if let Some(v) = f.network.orion_listen_port { cfg.orion_listen_port = v; }
        if let Some(v) = f.network.scene_generator_ip { cfg.scene_generator_ip = v; }
        if let Some(v) = f.network.scene_generator_cigi_port { cfg.scene_generator_cigi_port = v; }
        if let Some(v) = f.network.cigi_listen_port { cfg.cigi_listen_port = v; }

        // ── Platform source ──
        if let Some(k) = f.platform_source.kind { cfg.platform_source = k; }
        if let Some(v) = f.platform_source.mavlink.listen_port { cfg.mavlink_listen_port = v; }
        if let Some(v) = f.platform_source.mavlink.system_id { cfg.mavlink_system_id = v; }
        if let Some(v) = f.platform_source.stanag4586.listen_port { cfg.stanag_listen_port = v; }
        if let Some(v) = f.platform_source.stanag4586.multicast_group { cfg.stanag_multicast_group = v; }
        if let Some(v) = f.platform_source.stanag4586.vehicle_id_filter { cfg.stanag_vehicle_id = v; }
        if let Some(v) = f.platform_source.static_.lat_deg { cfg.platform_lat = v.to_radians(); }
        if let Some(v) = f.platform_source.static_.lon_deg { cfg.platform_lon = v.to_radians(); }
        if let Some(v) = f.platform_source.static_.alt_m { cfg.platform_alt = v; }
        if let Some(v) = f.platform_source.static_.roll_deg { cfg.platform_roll = v.to_radians(); }
        if let Some(v) = f.platform_source.static_.pitch_deg { cfg.platform_pitch = v.to_radians(); }
        if let Some(v) = f.platform_source.static_.yaw_deg { cfg.platform_yaw = v.to_radians(); }

        // ── Kinematics ──
        if let Some(v) = f.kinematics.max_slew_rate_deg_s { cfg.max_slew_rate = v.to_radians(); }
        if let Some(v) = f.kinematics.max_accel_deg_s2 { cfg.max_accel = v.to_radians(); }
        if let Some(v) = f.kinematics.pan_limit_deg { cfg.pan_limit = v.to_radians(); }
        if let Some(v) = f.kinematics.tilt_min_deg { cfg.tilt_min = v.to_radians(); }
        if let Some(v) = f.kinematics.tilt_max_deg { cfg.tilt_max = v.to_radians(); }

        // ── Camera ──
        // Per-camera entries override the defaults for each of the three
        // slots in `camera_table`. Missing entries keep the default.
        apply_camera_entry(&f.camera.eo_wide, &mut cfg.camera_table[0]);
        apply_camera_entry(&f.camera.eo_narrow, &mut cfg.camera_table[1]);
        apply_camera_entry(&f.camera.ir, &mut cfg.camera_table[2]);
        // Sync cam-0 into the legacy `hfov_wide` / etc. fields that
        // `fov_at_zoom` reads. These are kept as radians.
        cfg.hfov_wide = cfg.camera_table[0].0.to_radians();
        cfg.vfov_wide = cfg.camera_table[0].1.to_radians();
        cfg.hfov_narrow = cfg.camera_table[0].2.to_radians();
        cfg.vfov_narrow = cfg.camera_table[0].3.to_radians();

        // ── Vibration ──
        if let Some(v) = f.vibration.jitter_freq_hz { cfg.jitter_freq = v; }
        if let Some(v) = f.vibration.jitter_amplitude_deg { cfg.jitter_amplitude = v.to_radians(); }
        if let Some(v) = f.vibration.noise_floor_deg { cfg.noise_floor = v.to_radians(); }
        if let Some(v) = f.vibration.jitter_freq_2_hz { cfg.jitter_freq_2 = v; }
        if let Some(v) = f.vibration.jitter_amplitude_2_deg { cfg.jitter_amplitude_2 = v.to_radians(); }

        // ── Stabilization ──
        if let Some(v) = f.stabilization.quality { cfg.stabilization_quality = v.clamp(0.0, 1.0); }

        // ── Gimbal mount ──
        if let Some(v) = f.gimbal_mount.fwd_m   { cfg.gimbal_mount.translation_body_m[0] = v; }
        if let Some(v) = f.gimbal_mount.right_m { cfg.gimbal_mount.translation_body_m[1] = v; }
        if let Some(v) = f.gimbal_mount.down_m  { cfg.gimbal_mount.translation_body_m[2] = v; }
        if let Some(v) = f.gimbal_mount.roll_deg  { cfg.gimbal_mount.rotation_body_rad[0] = v.to_radians(); }
        if let Some(v) = f.gimbal_mount.pitch_deg { cfg.gimbal_mount.rotation_body_rad[1] = v.to_radians(); }
        if let Some(v) = f.gimbal_mount.yaw_deg   { cfg.gimbal_mount.rotation_body_rad[2] = v.to_radians(); }

        // ── Atmospheric / terrain / geopoint / track ──
        if let Some(v) = f.atmosphere.refraction_enabled { cfg.refraction_enabled = v; }
        if let Some(v) = f.terrain.elevation_m { cfg.terrain_elevation_m = v.max(0.0); }
        if let Some(v) = f.geopoint.alt_m { cfg.geopoint_alt_m = v; }
        if let Some(v) = f.track.p_gain { cfg.track_p_gain = v; }
        if let Some(v) = f.track.d_gain { cfg.track_d_gain = v; }
        if let Some(v) = f.track.loss_threshold { cfg.track_loss_threshold = v.clamp(0.0, 1.0); }
        if let Some(v) = f.track.gate_size_deg { cfg.track_gate_size_deg = v.max(0.0); }
        if let Some(v) = f.track.target_size_m { cfg.track_target_size_m = v.max(0.0); }

        // ── Laser / coupling ──
        if let Some(v) = f.laser.max_range_m { cfg.laser_max_range_m = v.max(0.0); }
        if let Some(v) = f.coupling.gyro_factor { cfg.gyro_coupling_factor = v; }

        cfg
    }
}

fn apply_camera_entry(src: &CameraEntrySection, dst: &mut (f32, f32, f32, f32)) {
    if let Some(v) = src.hfov_wide_deg { dst.0 = v; }
    if let Some(v) = src.vfov_wide_deg { dst.1 = v; }
    if let Some(v) = src.hfov_narrow_deg { dst.2 = v; }
    if let Some(v) = src.vfov_narrow_deg { dst.3 = v; }
}

impl Config {
    /// Load from `path`.
    ///
    /// - Missing file → `Config::default()`.
    /// - Malformed TOML or unknown keys → log the error and fall back to
    ///   defaults. `deny_unknown_fields` on every section means misspelled
    ///   keys surface as errors rather than silently doing nothing.
    pub fn load(path: &str) -> Self {
        let Ok(text) = std::fs::read_to_string(path) else {
            return Config::default();
        };
        match toml::from_str::<ConfigFile>(&text) {
            Ok(file) => file.into(),
            Err(e) => {
                eprintln!("[config] failed to parse {path}: {e}");
                Config::default()
            }
        }
    }

    /// Seed a `PlatformState` from the static position/attitude fields in the
    /// config. Kept as a small wrapper around
    /// [`PlatformSourceConfig::initial_state`] for backward compat with
    /// callers that don't want to materialise the full source enum.
    pub fn initial_platform_state(&self) -> sim_core::platform::PlatformState {
        self.platform_source_config().initial_state()
    }

    /// Package the config fields into the canonical
    /// [`PlatformSourceConfig`] variant. Called once at startup by `main.rs`.
    ///
    /// Unknown / missing `platform_source` strings fall through to the
    /// `Static` variant, which preserves the pre-Phase-3c behaviour.
    pub fn platform_source_config(&self) -> PlatformSourceConfig {
        match self.platform_source.as_str() {
            "mavlink" => PlatformSourceConfig::MavLink(MavLinkSourceConfig {
                listen_port: self.mavlink_listen_port,
                system_id: self.mavlink_system_id,
            }),
            "stanag4586" => PlatformSourceConfig::Stanag4586(Stanag4586SourceConfig {
                listen_port: self.stanag_listen_port,
                multicast_group: self.stanag_multicast_group.clone(),
                vehicle_id_filter: self.stanag_vehicle_id,
            }),
            _ => PlatformSourceConfig::Static(StaticSourceConfig {
                lat_rad: self.platform_lat,
                lon_rad: self.platform_lon,
                alt_m: self.platform_alt,
                roll_rad: self.platform_roll,
                pitch_rad: self.platform_pitch,
                yaw_rad: self.platform_yaw,
            }),
        }
    }

    /// Override the runtime mode (used by `main` to apply the CLI flag on
    /// top of the parsed config).
    pub fn with_runtime_mode(mut self, mode: RuntimeMode) -> Self {
        self.runtime_mode = mode;
        self
    }

    /// Returns `true` when pan rotation is continuous (no hard stop).
    ///
    /// Triggered by `pan_limit_deg >= 360` in the config file.
    pub fn is_continuous_pan(&self) -> bool {
        self.pan_limit >= std::f32::consts::TAU - 0.001 // ≥ ~359.9°
    }

    /// HFOV/VFOV interpolated by zoom level in [0, 1] (0 = wide, 1 = narrow).
    ///
    /// Uses a logarithmic (exponential) curve that models real optical zoom:
    /// `fov = fov_wide * (fov_narrow / fov_wide).powf(zoom)`.
    pub fn fov_at_zoom(&self, zoom: f32) -> (f32, f32) {
        let t = zoom.clamp(0.0, 1.0);
        let hfov = self.hfov_wide * (self.hfov_narrow / self.hfov_wide).powf(t);
        let vfov = self.vfov_wide * (self.vfov_narrow / self.vfov_wide).powf(t);
        (hfov, vfov)
    }

    /// Look up `(hfov_deg, vfov_deg)` wide-end FOV for a camera index.
    /// Falls back to camera 0 for out-of-range indices.
    ///
    /// Returns **degrees** because `camera_table` is stored in degrees for
    /// configuration ergonomics. `fov_at_zoom_for_camera` is the radian-valued
    /// counterpart used by the simulator and should be preferred on hot paths.
    pub fn camera_fov(&self, index: i8) -> (f32, f32) {
        let i = index.max(0) as usize;
        let entry = if i < self.camera_table.len() {
            self.camera_table[i]
        } else {
            self.camera_table[0]
        };
        (entry.0, entry.1)
    }

    /// HFOV/VFOV interpolated by zoom level for a specific camera.
    /// zoom 0 = wide end, 1 = narrow end. Returns radians.
    pub fn fov_at_zoom_for_camera(&self, camera: i8, zoom: f32) -> (f32, f32) {
        let i = camera.max(0) as usize;
        let entry = if i < self.camera_table.len() {
            self.camera_table[i]
        } else {
            self.camera_table[0]
        };
        let t = zoom.clamp(0.0, 1.0);
        let hfov = entry.0 * (entry.2 / entry.0).powf(t);
        let vfov = entry.1 * (entry.3 / entry.1).powf(t);
        (hfov.to_radians(), vfov.to_radians())
    }
}

/// Legacy free function — delegates to default config.
/// Prefer `Config::camera_fov()` for per-config camera tables.
pub fn camera_fov(index: i8) -> (f32, f32) {
    Config::default().camera_fov(index)
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

    // ── Runtime mode ─────────────────────────────────────────────────────────

    #[test]
    fn load_runtime_section_simulator() {
        let content = "[runtime]\nmode = \"simulator\"\n";
        let path = write_temp("runtime_sim", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);
        assert_eq!(cfg.runtime_mode, RuntimeMode::Simulator);
    }

    #[test]
    fn load_runtime_section_bridge_explicit() {
        let content = "[runtime]\nmode = \"bridge\"\n";
        let path = write_temp("runtime_bridge", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);
        assert_eq!(cfg.runtime_mode, RuntimeMode::Bridge);
    }

    #[test]
    fn load_missing_runtime_defaults_to_bridge() {
        let content = "[network]\norion_listen_port = 9090\n";
        let path = write_temp("runtime_missing", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);
        assert_eq!(cfg.runtime_mode, RuntimeMode::Bridge);
    }

    #[test]
    fn load_runtime_unknown_mode_falls_back_to_bridge() {
        // Unknown mode strings are accepted at the TOML layer (the field is
        // an Option<String>) but produce a stderr warning and leave the
        // default in place.
        let content = "[runtime]\nmode = \"zorp\"\n";
        let path = write_temp("runtime_unknown", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);
        assert_eq!(cfg.runtime_mode, RuntimeMode::Bridge);
    }

    #[test]
    fn load_runtime_denies_unknown_fields() {
        // A typo inside the [runtime] section must fail parsing (and fall
        // through to a full default), thanks to deny_unknown_fields.
        let content = "[runtime]\nmodes = \"simulator\"\n";
        let path = write_temp("runtime_typo", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);
        assert_eq!(cfg.runtime_mode, RuntimeMode::Bridge);
    }

    #[test]
    fn with_runtime_mode_overrides() {
        let cfg = Config::default().with_runtime_mode(RuntimeMode::Simulator);
        assert_eq!(cfg.runtime_mode, RuntimeMode::Simulator);
        let cfg2 = cfg.with_runtime_mode(RuntimeMode::Bridge);
        assert_eq!(cfg2.runtime_mode, RuntimeMode::Bridge);
    }

    #[test]
    fn runtime_mode_parse_case_insensitive() {
        assert_eq!(RuntimeMode::parse("SIMULATOR"), Some(RuntimeMode::Simulator));
        assert_eq!(RuntimeMode::parse("Bridge"), Some(RuntimeMode::Bridge));
        assert_eq!(RuntimeMode::parse("sim"), Some(RuntimeMode::Simulator));
        assert_eq!(RuntimeMode::parse(" bridge "), Some(RuntimeMode::Bridge));
        assert_eq!(RuntimeMode::parse("zorp"), None);
    }

    // ── Config::load ─────────────────────────────────────────────────────────

    #[test]
    fn load_missing_file_returns_default() {
        let cfg = Config::load("/tmp/cigi_trillium_no_such_file_xyz999.toml");
        let def = Config::default();
        assert_eq!(cfg.orion_listen_port, def.orion_listen_port);
        assert_eq!(cfg.scene_generator_ip, def.scene_generator_ip);
        assert_eq!(cfg.max_slew_rate, def.max_slew_rate);
    }

    #[test]
    fn load_network_section() {
        let content = concat!(
            "[network]\n",
            "orion_listen_port = 9090\n",
            "scene_generator_ip = \"192.168.1.50\"\n",
            "scene_generator_cigi_port = 18888\n",
            "cigi_listen_port = 18889\n",
        );
        let path = write_temp("network", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        assert_eq!(cfg.orion_listen_port, 9090);
        assert_eq!(cfg.scene_generator_ip, "192.168.1.50");
        assert_eq!(cfg.scene_generator_cigi_port, 18888);
        assert_eq!(cfg.cigi_listen_port, 18889);
    }

    #[test]
    fn load_kinematics_section_converts_deg_to_rad() {
        let content = concat!(
            "[kinematics]\n",
            "max_slew_rate_deg_s = 120.0\n",
            "pan_limit_deg = 180.0\n",
        );
        let path = write_temp("kinematics", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        assert!((cfg.max_slew_rate - 120.0_f32.to_radians()).abs() < 1e-6);
        assert!((cfg.pan_limit - 180.0_f32.to_radians()).abs() < 1e-6);
    }

    #[test]
    fn load_unknown_keys_are_rejected() {
        // With deny_unknown_fields, a typo is a hard error rather than a
        // silent no-op. Parse failures fall back to Config::default.
        let content = concat!(
            "[network]\n",
            "totaly_wrong_key = 999\n",
        );
        let path = write_temp("unknown", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        // Fell back to default because parsing failed.
        let def = Config::default();
        assert_eq!(cfg.orion_listen_port, def.orion_listen_port);
    }

    #[test]
    fn load_partial_section_keeps_defaults_for_missing_keys() {
        let content = concat!(
            "[network]\n",
            "orion_listen_port = 1234\n",
            // scene_generator_ip missing — default should survive
        );
        let path = write_temp("partial_network", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        assert_eq!(cfg.orion_listen_port, 1234);
        assert_eq!(cfg.scene_generator_ip, "127.0.0.1");
    }

    #[test]
    fn load_static_platform_section_converts_lat_lon() {
        let content = concat!(
            "[platform_source]\n",
            "kind = \"static\"\n",
            "[platform_source.static]\n",
            "lat_deg = 90.0\n",
            "lon_deg = -45.0\n",
            "alt_m = 1500.0\n",
        );
        let path = write_temp("static_platform", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        assert!((cfg.platform_lat - std::f64::consts::FRAC_PI_2).abs() < 1e-10);
        assert!((cfg.platform_lon + std::f64::consts::FRAC_PI_4).abs() < 1e-10);
        assert!((cfg.platform_alt - 1500.0).abs() < 1e-9);
    }

    #[test]
    fn load_mavlink_subtable() {
        let content = concat!(
            "[platform_source]\n",
            "kind = \"mavlink\"\n",
            "[platform_source.mavlink]\n",
            "listen_port = 14599\n",
            "system_id = 255\n",
        );
        let path = write_temp("mavlink_sub", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        assert_eq!(cfg.platform_source, "mavlink");
        assert_eq!(cfg.mavlink_listen_port, 14599);
        assert_eq!(cfg.mavlink_system_id, 255_u8);
    }

    #[test]
    fn load_stanag_subtable() {
        let content = concat!(
            "[platform_source]\n",
            "kind = \"stanag4586\"\n",
            "[platform_source.stanag4586]\n",
            "listen_port = 4599\n",
            "multicast_group = \"239.0.0.42\"\n",
            "vehicle_id_filter = -1\n",
        );
        let path = write_temp("stanag_sub", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        assert_eq!(cfg.platform_source, "stanag4586");
        assert_eq!(cfg.stanag_listen_port, 4599);
        assert_eq!(cfg.stanag_multicast_group, "239.0.0.42");
        assert_eq!(cfg.stanag_vehicle_id, -1);
    }

    #[test]
    fn load_gimbal_mount_all_fields() {
        let content = concat!(
            "[gimbal_mount]\n",
            "fwd_m = 0.8\n",
            "right_m = -0.05\n",
            "down_m = 0.15\n",
            "roll_deg = 0.0\n",
            "pitch_deg = -2.5\n",
            "yaw_deg = 1.0\n",
        );
        let path = write_temp("gimbal_mount", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        let m = cfg.gimbal_mount;
        assert!((m.translation_body_m[0] - 0.8).abs() < 1e-9);
        assert!((m.translation_body_m[1] + 0.05).abs() < 1e-9);
        assert!((m.translation_body_m[2] - 0.15).abs() < 1e-9);
        assert!((m.rotation_body_rad[0]).abs() < 1e-9);
        assert!((m.rotation_body_rad[1] - (-2.5_f64).to_radians()).abs() < 1e-12);
        assert!((m.rotation_body_rad[2] - 1.0_f64.to_radians()).abs() < 1e-12);
    }

    #[test]
    fn load_gimbal_mount_default_is_identity() {
        let cfg = Config::default();
        assert_eq!(cfg.gimbal_mount, GimbalMount::default());
        assert_eq!(cfg.gimbal_mount.translation_body_m, [0.0; 3]);
        assert_eq!(cfg.gimbal_mount.rotation_body_rad, [0.0; 3]);
    }

    #[test]
    fn platform_source_config_mavlink_variant() {
        let content = concat!(
            "[platform_source]\n",
            "kind = \"mavlink\"\n",
            "[platform_source.mavlink]\n",
            "listen_port = 14599\n",
            "system_id = 42\n",
        );
        let path = write_temp("psc_mavlink", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        let psc = cfg.platform_source_config();
        assert_eq!(psc.kind(), "mavlink");
        match psc {
            PlatformSourceConfig::MavLink(c) => {
                assert_eq!(c.listen_port, 14599);
                assert_eq!(c.system_id, 42);
            }
            _ => panic!("expected MavLink variant"),
        }
    }

    #[test]
    fn platform_source_config_stanag_variant() {
        let content = concat!(
            "[platform_source]\n",
            "kind = \"stanag4586\"\n",
            "[platform_source.stanag4586]\n",
            "listen_port = 4587\n",
            "multicast_group = \"239.0.0.99\"\n",
            "vehicle_id_filter = 7\n",
        );
        let path = write_temp("psc_stanag", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        let psc = cfg.platform_source_config();
        assert_eq!(psc.kind(), "stanag4586");
        match psc {
            PlatformSourceConfig::Stanag4586(c) => {
                assert_eq!(c.listen_port, 4587);
                assert_eq!(c.multicast_group, "239.0.0.99");
                assert_eq!(c.vehicle_id_filter, 7);
            }
            _ => panic!("expected Stanag4586 variant"),
        }
    }

    #[test]
    fn platform_source_config_static_variant_from_unknown_kind() {
        let content = concat!(
            "[platform_source]\n",
            "kind = \"static\"\n",
            "[platform_source.static]\n",
            "lat_deg = 37.5\n",
            "lon_deg = -122.1\n",
            "alt_m = 1500.0\n",
        );
        let path = write_temp("psc_static", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        let psc = cfg.platform_source_config();
        assert_eq!(psc.kind(), "static");
        let state = psc.initial_state();
        assert!((state.lat_rad - 37.5_f64.to_radians()).abs() < 1e-10);
        assert!((state.lon_rad + 122.1_f64.to_radians()).abs() < 1e-10);
        assert!((state.alt_m - 1500.0).abs() < 1e-9);
    }

    // ── Config::is_continuous_pan ─────────────────────────────────────────────

    #[test]
    fn is_continuous_pan_default_is_false() {
        assert!(!Config::default().is_continuous_pan());
    }

    #[test]
    fn is_continuous_pan_true_at_360_deg() {
        let content = "[kinematics]\npan_limit_deg = 360.0\n";
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
    fn fov_at_zoom_half_is_geometric_mean() {
        let cfg = Config::default();
        let (h, v) = cfg.fov_at_zoom(0.5);
        let h_geo = (cfg.hfov_wide * cfg.hfov_narrow).sqrt();
        let v_geo = (cfg.vfov_wide * cfg.vfov_narrow).sqrt();
        assert!((h - h_geo).abs() < 1e-6);
        assert!((v - v_geo).abs() < 1e-6);
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
        assert_eq!(Config::default().camera_fov(0), (30.0, 22.5));
    }

    #[test]
    fn camera_fov_index_1() {
        assert_eq!(Config::default().camera_fov(1), (5.0, 3.75));
    }

    #[test]
    fn camera_fov_index_2() {
        assert_eq!(Config::default().camera_fov(2), (20.0, 15.0));
    }

    #[test]
    fn camera_fov_out_of_range_falls_back_to_cam0() {
        let cfg = Config::default();
        assert_eq!(cfg.camera_fov(99), cfg.camera_fov(0));
    }

    #[test]
    fn camera_fov_free_function_uses_defaults() {
        assert_eq!(camera_fov(0), (30.0, 22.5));
        assert_eq!(camera_fov(1), (5.0, 3.75));
        assert_eq!(camera_fov(2), (20.0, 15.0));
    }

    // ── per-camera config ────────────────────────────────────────────────────

    #[test]
    fn per_camera_fov_from_config_file() {
        let content = concat!(
            "[camera.eo_wide]\n",
            "hfov_wide_deg = 46.8\n",
            "vfov_wide_deg = 35.1\n",
            "hfov_narrow_deg = 1.2\n",
            "vfov_narrow_deg = 0.9\n",
            "[camera.ir]\n",
            "hfov_wide_deg = 29.0\n",
            "vfov_wide_deg = 21.75\n",
            "hfov_narrow_deg = 3.0\n",
            "vfov_narrow_deg = 2.25\n",
        );
        let path = write_temp("per_camera_fov", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        assert_eq!(cfg.camera_fov(0), (46.8, 35.1));
        // eo_narrow not specified → default
        assert_eq!(cfg.camera_fov(1), (5.0, 3.75));
        assert_eq!(cfg.camera_fov(2), (29.0, 21.75));
    }

    #[test]
    fn per_camera_zoom_interpolation() {
        let content = concat!(
            "[camera.ir]\n",
            "hfov_wide_deg = 29.0\n",
            "vfov_wide_deg = 21.75\n",
            "hfov_narrow_deg = 3.0\n",
            "vfov_narrow_deg = 2.25\n",
        );
        let path = write_temp("per_camera_zoom", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        let (h, _) = cfg.fov_at_zoom_for_camera(2, 0.0);
        assert!((h - 29.0_f32.to_radians()).abs() < 1e-5);

        let (h, _) = cfg.fov_at_zoom_for_camera(2, 1.0);
        assert!((h - 3.0_f32.to_radians()).abs() < 1e-5);

        let (h, _) = cfg.fov_at_zoom_for_camera(2, 0.5);
        let expected: f32 = (29.0_f32 * 3.0_f32).sqrt();
        assert!((h - expected.to_radians()).abs() < 1e-5);
    }

    #[test]
    fn per_camera_eo_wide_section_sets_legacy_hfov_fields() {
        // [camera.eo_wide] should populate both camera_table[0] AND the
        // legacy hfov_wide/narrow fields used by fov_at_zoom().
        let content = concat!(
            "[camera.eo_wide]\n",
            "hfov_wide_deg = 50.0\n",
            "vfov_wide_deg = 37.5\n",
            "hfov_narrow_deg = 5.0\n",
            "vfov_narrow_deg = 3.75\n",
        );
        let path = write_temp("legacy_cam0_sync", content);
        let cfg = Config::load(&path);
        let _ = fs::remove_file(&path);

        assert_eq!(cfg.camera_fov(0), (50.0, 37.5));
        assert!((cfg.hfov_wide - 50.0_f32.to_radians()).abs() < 1e-5);
    }
}
