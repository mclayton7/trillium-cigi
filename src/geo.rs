// WGS84 geolocation math for gimbal line-of-sight projection and geopoint inverse.
//
// Phase 3.1 — WGS84 LOS projection
// Phase 3.2 — Geopoint inverse (pan/tilt from target lat/lon)

/// WGS84 semi-major axis (metres).
const A: f64 = 6_378_137.0;
/// WGS84 semi-minor axis (metres).
const B: f64 = 6_356_752.314_245;
/// WGS84 first eccentricity squared.
const E2: f64 = 1.0 - (B * B) / (A * A);
/// Mean Earth radius for NED velocity approximation (metres).
const RE: f64 = 6_371_000.0;

// ─────────────────────────────────────────── ECEF ──

/// Convert geodetic (lat, lon in **radians**, alt in metres) → ECEF (metres).
pub fn geodetic_to_ecef(lat: f64, lon: f64, alt: f64) -> [f64; 3] {
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let sin_lon = lon.sin();
    let cos_lon = lon.cos();
    let n = A / (1.0 - E2 * sin_lat * sin_lat).sqrt();
    [
        (n + alt) * cos_lat * cos_lon,
        (n + alt) * cos_lat * sin_lon,
        (n * (1.0 - E2) + alt) * sin_lat,
    ]
}

/// Convert ECEF (metres) → geodetic [lat_rad, lon_rad, alt_m] using iterative Bowring.
pub fn ecef_to_geodetic(x: f64, y: f64, z: f64) -> [f64; 3] {
    let lon = y.atan2(x);
    let p = (x * x + y * y).sqrt();
    let mut lat = z.atan2(p * (1.0 - E2));
    for _ in 0..5 {
        let sin_lat = lat.sin();
        let n = A / (1.0 - E2 * sin_lat * sin_lat).sqrt();
        lat = (z + E2 * n * sin_lat).atan2(p);
    }
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let n = A / (1.0 - E2 * sin_lat * sin_lat).sqrt();
    let alt = if cos_lat.abs() > 1e-10 { p / cos_lat - n } else { z.abs() / sin_lat.abs() - n * (1.0 - E2) };
    [lat, lon, alt]
}

// ─────────────────────────────────────────── NED frame ──

/// Compute NED basis unit vectors at a geodetic position, expressed in ECEF.
/// Returns (north_hat, east_hat, down_hat).
pub fn ned_frame(lat: f64, lon: f64) -> ([f64; 3], [f64; 3], [f64; 3]) {
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let sin_lon = lon.sin();
    let cos_lon = lon.cos();
    let north = [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat];
    let east  = [-sin_lon,            cos_lon,            0.0];
    let down  = [-cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat];
    (north, east, down)
}

// ─────────────────────────────────────────── LOS ──

/// Compute the gimbal line-of-sight as an ECEF unit vector.
///
/// `pan` is the azimuth in the gimbal body frame (0 = nose, + clockwise), radians.
/// `tilt` is the depression angle (0 = horizontal, + downward), radians.
/// Platform attitude (roll/pitch/yaw) rotates the gimbal-frame direction into NED.
/// For a fully stabilised gimbal (`stabilization_quality = 0`) only yaw is applied;
/// at `stabilization_quality = 1` the full ZYX rotation (yaw, pitch, roll) applies.
///
/// `platform_roll` and `platform_pitch` are applied scaled by
/// `stabilization_quality` (0.0 = perfect stabilization, 1.0 = body-mounted).
/// The rotation order is Rz(yaw) · Ry(pitch·sq) · Rx(roll·sq) applied to the
/// gimbal-frame LOS direction.
pub fn los_ecef(
    lat: f64,
    lon: f64,
    platform_yaw: f32,
    platform_roll: f32,
    platform_pitch: f32,
    stabilization_quality: f32,
    pan: f32,
    tilt: f32,
) -> [f32; 3] {
    // Gimbal-frame LOS: pan is azimuth from nose, tilt is depression.
    let az = pan as f64;
    let el = tilt as f64; // positive = depression (downward)

    // LOS in gimbal body frame (forward = +X, right = +Y, down = +Z convention
    // mapped to NED: north, east, down).
    let n_body = el.cos() * az.cos();
    let e_body = el.cos() * az.sin();
    let d_body = el.sin(); // positive down

    // Apply platform attitude rotation: Rz(yaw) * Ry(pitch*sq) * Rx(roll*sq)
    let sq = stabilization_quality as f64;
    let roll  = (platform_roll  as f64) * sq;
    let pitch = (platform_pitch as f64) * sq;
    let yaw   = platform_yaw as f64;

    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();

    // Rz(yaw) * Ry(pitch) * Rx(roll) rotation matrix applied to [n_body, e_body, d_body]:
    // Row 0: [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr]
    // Row 1: [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr]
    // Row 2: [-sp,    cp*sr,             cp*cr            ]
    let n = (cy * cp) * n_body + (cy * sp * sr - sy * cr) * e_body + (cy * sp * cr + sy * sr) * d_body;
    let e = (sy * cp) * n_body + (sy * sp * sr + cy * cr) * e_body + (sy * sp * cr - cy * sr) * d_body;
    let d = (-sp)     * n_body + (cp * sr)                * e_body + (cp * cr)                * d_body;

    let (n_hat, e_hat, d_hat) = ned_frame(lat, lon);
    let x = n_hat[0] * n + e_hat[0] * e + d_hat[0] * d;
    let y = n_hat[1] * n + e_hat[1] * e + d_hat[1] * d;
    let z = n_hat[2] * n + e_hat[2] * e + d_hat[2] * d;
    [x as f32, y as f32, z as f32]
}

// ─────────────────────────────────────────── Ray-cast ──

/// Find where a ray from `origin` (ECEF, metres) in direction `dir` (unit vector)
/// intersects the WGS84 ellipsoid.  Returns the closer positive-t intersection.
pub fn ray_wgs84(origin: [f64; 3], dir: [f64; 3]) -> Option<[f64; 3]> {
    let [ox, oy, oz] = origin;
    let [dx, dy, dz] = dir;
    let a2 = A * A;
    let b2 = B * B;
    let qa = (dx * dx + dy * dy) / a2 + dz * dz / b2;
    let qb = 2.0 * ((ox * dx + oy * dy) / a2 + oz * dz / b2);
    let qc = (ox * ox + oy * oy) / a2 + oz * oz / b2 - 1.0;
    let disc = qb * qb - 4.0 * qa * qc;
    if disc < 0.0 {
        return None;
    }
    let t1 = (-qb - disc.sqrt()) / (2.0 * qa);
    let t2 = (-qb + disc.sqrt()) / (2.0 * qa);
    let t = if t1 > 0.001 { t1 } else if t2 > 0.001 { t2 } else { return None; };
    let px = ox + t * dx;
    let py = oy + t * dy;
    let pz = oz + t * dz;
    Some(ecef_to_geodetic(px, py, pz))
}

// ─────────────────────────────────────────── Public API ──

/// Given platform position and stabilised gimbal angles, compute the ground look-point.
///
/// Returns `[lat_rad, lon_rad, alt_m]` on the WGS84 ellipsoid, or `None` if the
/// line of sight does not intersect the Earth (pointing above the horizon).
/// Bennett's formula for atmospheric refraction (simplified for EO/IR).
///
/// Given a true elevation angle `elev` in radians (negative = below horizon),
/// returns the refraction offset in radians (always >= 0). The correction
/// bends the apparent LOS downward, making the effective depression shallower
/// (i.e. the look-point moves further away).
fn bennett_refraction(elev: f64) -> f64 {
    // Bennett's approximation; arguments in radians.
    // refraction = 0.0002967 / tan(elev + 0.00312 / (elev + 0.089))
    let denom = (elev + 0.00312 / (elev + 0.089)).tan();
    if denom.abs() < 1e-12 {
        return 0.0;
    }
    (0.0002967 / denom).max(0.0)
}

pub fn compute_look_point(
    pos_lat: f64,
    pos_lon: f64,
    pos_alt: f64,
    platform_yaw: f32,
    platform_roll: f32,
    platform_pitch: f32,
    stabilization_quality: f32,
    pan: f32,
    tilt: f32,
    refraction_enabled: bool,
) -> Option<[f64; 3]> {
    if pos_alt < 1.0 {
        return None; // on the ground, no look-point
    }

    // Apply atmospheric refraction correction to the tilt angle.
    // Tilt convention: positive = depression (looking down).
    // Elevation angle = -tilt (positive = above horizon, negative = below).
    let effective_tilt = if refraction_enabled {
        let elev = -(tilt as f64); // elevation: negative when looking down
        let refraction_rad = bennett_refraction(elev);
        // Refraction makes objects appear higher → effective depression is less →
        // subtract from tilt (look further away).
        tilt - refraction_rad as f32
    } else {
        tilt
    };

    let origin = geodetic_to_ecef(pos_lat, pos_lon, pos_alt);
    let dir_f32 = los_ecef(pos_lat, pos_lon, platform_yaw, platform_roll, platform_pitch, stabilization_quality, pan, effective_tilt);
    let dir = [dir_f32[0] as f64, dir_f32[1] as f64, dir_f32[2] as f64];
    let len = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]).sqrt();
    if len < 1e-10 {
        return None;
    }
    let dir_n = [dir[0] / len, dir[1] / len, dir[2] / len];
    ray_wgs84(origin, dir_n)
}

/// Inverse geolocation: compute the pan/tilt angles needed to point at
/// `(target_lat, target_lon, target_alt)` from the platform position.
///
/// Returns `(pan_rad, tilt_rad)` in the platform-yaw-relative frame.
// NOTE: This function does not account for platform roll/pitch when
// stabilization_quality > 0. The forward LOS path (los_ecef) applies
// full ZYX rotation, but this inverse only subtracts yaw. Geopoint
// mode will have residual pointing error proportional to
// roll/pitch * stabilization_quality.
pub fn inverse_geopoint(
    pos_lat: f64,
    pos_lon: f64,
    pos_alt: f64,
    platform_yaw: f32,
    target_lat: f64,
    target_lon: f64,
    target_alt: f64,
) -> (f32, f32) {
    let origin = geodetic_to_ecef(pos_lat, pos_lon, pos_alt);
    let target = geodetic_to_ecef(target_lat, target_lon, target_alt);
    let dx = target[0] - origin[0];
    let dy = target[1] - origin[1];
    let dz = target[2] - origin[2];
    let len = (dx * dx + dy * dy + dz * dz).sqrt();
    if len < 1.0 {
        return (0.0, 0.0);
    }
    let dir = [dx / len, dy / len, dz / len];
    let (n_hat, e_hat, d_hat) = ned_frame(pos_lat, pos_lon);
    let n = dot3(dir, n_hat);
    let e = dot3(dir, e_hat);
    let d = dot3(dir, d_hat);
    // Azimuth in NED (0 = north, + clockwise)
    let az_ned = e.atan2(n);
    // Depression angle (+ down)
    let horiz = (n * n + e * e).sqrt();
    let tilt = d.atan2(horiz) as f32;
    // Subtract platform yaw to get gimbal-frame pan
    let pan = (az_ned as f32) - platform_yaw;
    (pan, tilt)
}

/// Approximate NED velocity (m/s) from two geodetic positions and a time delta.
pub fn ned_velocity(
    lat1: f64, lon1: f64, alt1: f64,
    lat2: f64, lon2: f64, alt2: f64,
    dt: f64,
) -> [f32; 3] {
    if dt < 1e-9 {
        return [0.0; 3];
    }
    let dlon = {
        let raw = lon2 - lon1;
        let pi2 = std::f64::consts::PI * 2.0;
        ((raw + std::f64::consts::PI).rem_euclid(pi2)) - std::f64::consts::PI
    };
    let mid_lat = (lat1 + lat2) * 0.5;
    let dn = (lat2 - lat1) * RE / dt;
    let de = dlon * RE * mid_lat.cos() / dt;
    let dd = -(alt2 - alt1) / dt;
    [dn as f32, de as f32, dd as f32]
}

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

// ─────────────────────────────────────────── Tests ──

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ecef_roundtrip() {
        let lat = 37.0_f64.to_radians();
        let lon = -122.0_f64.to_radians();
        let alt = 1000.0;
        let [x, y, z] = geodetic_to_ecef(lat, lon, alt);
        let [lat2, lon2, alt2] = ecef_to_geodetic(x, y, z);
        assert!((lat - lat2).abs() < 1e-10, "lat roundtrip: {} vs {}", lat, lat2);
        assert!((lon - lon2).abs() < 1e-10, "lon roundtrip: {} vs {}", lon, lon2);
        assert!((alt - alt2).abs() < 0.01, "alt roundtrip: {} vs {}", alt, alt2);
    }

    #[test]
    fn look_point_nadir() {
        // Straight down should land directly below
        let lat = 37.0_f64.to_radians();
        let lon = -122.0_f64.to_radians();
        let alt = 1000.0;
        let result = compute_look_point(lat, lon, alt, 0.0, 0.0, 0.0, 0.0, 0.0, std::f32::consts::FRAC_PI_2, false);
        let [rl, ro, ra] = result.expect("should intersect");
        assert!((rl - lat).abs() < 1e-5, "look lat should match platform lat");
        assert!((ro - lon).abs() < 1e-5, "look lon should match platform lon");
        assert!(ra < 10.0, "look alt should be near sea level");
    }

    #[test]
    fn inverse_geopoint_roundtrip() {
        let pos_lat = 37.0_f64.to_radians();
        let pos_lon = -122.0_f64.to_radians();
        let pos_alt = 1000.0;
        // Compute look point forward and to the right
        let pan0 = 0.5_f32;
        let tilt0 = 0.7_f32; // 40° depression
        if let Some([tl, to, ta]) = compute_look_point(pos_lat, pos_lon, pos_alt, 0.0, 0.0, 0.0, 0.0, pan0, tilt0, false) {
            let (pan1, tilt1) = inverse_geopoint(pos_lat, pos_lon, pos_alt, 0.0, tl, to, ta);
            assert!((pan1 - pan0).abs() < 0.01, "pan inverse: {} vs {}", pan1, pan0);
            assert!((tilt1 - tilt0).abs() < 0.01, "tilt inverse: {} vs {}", tilt1, tilt0);
        }
    }

    #[test]
    fn inverse_geopoint_nonzero_altitude() {
        let pos_lat = 37.0_f64.to_radians();
        let pos_lon = -122.0_f64.to_radians();
        let pos_alt = 2000.0;
        let target_lat = 37.001_f64.to_radians();
        let target_lon = -121.999_f64.to_radians();

        // At sea level
        let (pan0, tilt0) = inverse_geopoint(pos_lat, pos_lon, pos_alt, 0.0, target_lat, target_lon, 0.0);
        // At 500 m altitude
        let (pan500, tilt500) = inverse_geopoint(pos_lat, pos_lon, pos_alt, 0.0, target_lat, target_lon, 500.0);

        // Pan should be nearly identical (same horizontal direction).
        assert!((pan0 - pan500).abs() < 0.01, "pan should be similar: {} vs {}", pan0, pan500);
        // Higher target → less depression (smaller tilt).
        assert!(tilt500 < tilt0, "higher target alt should reduce tilt: {} vs {}", tilt500, tilt0);
    }

    #[test]
    fn stabilization_quality_zero_matches_yaw_only() {
        // With stab_quality=0.0, roll and pitch should have no effect,
        // matching the old yaw-only behavior.
        let lat = 37.0_f64.to_radians();
        let lon = -122.0_f64.to_radians();
        let alt = 1000.0;
        let yaw = 0.3_f32;
        let pan = 0.5_f32;
        let tilt = 0.7_f32;

        let result_no_rp = compute_look_point(lat, lon, alt, yaw, 0.0, 0.0, 0.0, pan, tilt, false);
        // Non-zero roll/pitch but stab_quality = 0.0 → should be identical
        let result_with_rp = compute_look_point(lat, lon, alt, yaw, 0.2, -0.1, 0.0, pan, tilt, false);
        let a = result_no_rp.expect("should intersect");
        let b = result_with_rp.expect("should intersect");
        assert!((a[0] - b[0]).abs() < 1e-10, "lat should match: {} vs {}", a[0], b[0]);
        assert!((a[1] - b[1]).abs() < 1e-10, "lon should match: {} vs {}", a[1], b[1]);
        assert!((a[2] - b[2]).abs() < 0.01, "alt should match: {} vs {}", a[2], b[2]);
    }

    #[test]
    fn stabilization_quality_one_shifts_look_point_with_roll() {
        // With stab_quality=1.0 and nonzero roll, the look-point should differ
        // from the zero-roll case.
        let lat = 37.0_f64.to_radians();
        let lon = -122.0_f64.to_radians();
        let alt = 1000.0;
        let yaw = 0.0_f32;
        let pan = 0.0_f32;
        let tilt = std::f32::consts::FRAC_PI_4; // 45° depression

        let result_no_roll = compute_look_point(lat, lon, alt, yaw, 0.0, 0.0, 1.0, pan, tilt, false);
        // Apply 10° roll
        let roll = 10.0_f32.to_radians();
        let result_with_roll = compute_look_point(lat, lon, alt, yaw, roll, 0.0, 1.0, pan, tilt, false);

        let a = result_no_roll.expect("should intersect");
        let b = result_with_roll.expect("should intersect");
        // The look-point should have shifted (lat or lon differs meaningfully)
        let dlat = (a[0] - b[0]).abs();
        let dlon = (a[1] - b[1]).abs();
        let shift = dlat + dlon;
        assert!(shift > 1e-6, "look-point should shift with roll: dlat={}, dlon={}", dlat, dlon);
    }

    #[test]
    fn stabilization_quality_one_shifts_look_point_with_pitch() {
        // With stab_quality=1.0 and nonzero pitch, the look-point should differ.
        let lat = 37.0_f64.to_radians();
        let lon = -122.0_f64.to_radians();
        let alt = 1000.0;
        let yaw = 0.0_f32;
        let pan = 0.0_f32;
        let tilt = std::f32::consts::FRAC_PI_4;

        let result_no_pitch = compute_look_point(lat, lon, alt, yaw, 0.0, 0.0, 1.0, pan, tilt, false);
        let pitch = 5.0_f32.to_radians();
        let result_with_pitch = compute_look_point(lat, lon, alt, yaw, 0.0, pitch, 1.0, pan, tilt, false);

        let a = result_no_pitch.expect("should intersect");
        let b = result_with_pitch.expect("should intersect");
        let dlat = (a[0] - b[0]).abs();
        let dlon = (a[1] - b[1]).abs();
        let shift = dlat + dlon;
        assert!(shift > 1e-6, "look-point should shift with pitch: dlat={}, dlon={}", dlat, dlon);
    }

    #[test]
    fn stabilization_quality_half_partial_effect() {
        // stab_quality=0.5 should produce a shift that is less than stab_quality=1.0
        let lat = 37.0_f64.to_radians();
        let lon = -122.0_f64.to_radians();
        let alt = 1000.0;
        let roll = 10.0_f32.to_radians();
        let pan = 0.0_f32;
        let tilt = std::f32::consts::FRAC_PI_4;

        let base = compute_look_point(lat, lon, alt, 0.0, 0.0, 0.0, 0.0, pan, tilt, false)
            .expect("intersect");
        let half = compute_look_point(lat, lon, alt, 0.0, roll, 0.0, 0.5, pan, tilt, false)
            .expect("intersect");
        let full = compute_look_point(lat, lon, alt, 0.0, roll, 0.0, 1.0, pan, tilt, false)
            .expect("intersect");

        let shift_half = (base[0] - half[0]).abs() + (base[1] - half[1]).abs();
        let shift_full = (base[0] - full[0]).abs() + (base[1] - full[1]).abs();
        assert!(shift_half > 1e-7, "half quality should produce a shift");
        assert!(shift_full > shift_half, "full quality should shift more than half: {} vs {}", shift_full, shift_half);
    }

    #[test]
    fn refraction_shallow_angle_shifts_look_point_further() {
        // At a shallow depression angle, refraction should push the look-point
        // further away (lower latitude difference from nadir for a northward look).
        let lat = 37.0_f64.to_radians();
        let lon = -122.0_f64.to_radians();
        let alt = 5000.0; // 5 km altitude for meaningful shallow-angle effect
        let pan = 0.0_f32; // looking north
        let tilt = 0.05_f32; // very shallow depression (~2.9°)

        let without = compute_look_point(lat, lon, alt, 0.0, 0.0, 0.0, 0.0, pan, tilt, false)
            .expect("should intersect without refraction");
        let with = compute_look_point(lat, lon, alt, 0.0, 0.0, 0.0, 0.0, pan, tilt, true)
            .expect("should intersect with refraction");

        // Refraction reduces effective depression → ray hits ground further away.
        // Looking north at shallow angle: the refracted look-point should have
        // a larger latitude (further north) than the unrefracted one.
        assert!(
            with[0] > without[0],
            "refraction should shift look-point further north: with={} vs without={}",
            with[0].to_degrees(), without[0].to_degrees()
        );
    }

    #[test]
    fn refraction_disabled_matches_original() {
        // With refraction disabled, compute_look_point should produce the
        // exact same result regardless of whether the flag existed before.
        let lat = 37.0_f64.to_radians();
        let lon = -122.0_f64.to_radians();
        let alt = 1000.0;
        let pan = 0.3_f32;
        let tilt = 0.5_f32;

        let a = compute_look_point(lat, lon, alt, 0.0, 0.0, 0.0, 0.0, pan, tilt, false);
        let b = compute_look_point(lat, lon, alt, 0.0, 0.0, 0.0, 0.0, pan, tilt, false);
        assert_eq!(a, b, "two calls with refraction disabled should be identical");

        // Also verify refraction enabled at steep angle produces negligible difference.
        let steep_tilt = std::f32::consts::FRAC_PI_2; // 90° straight down
        let steep_off = compute_look_point(lat, lon, alt, 0.0, 0.0, 0.0, 0.0, 0.0, steep_tilt, false)
            .expect("intersect");
        let steep_on = compute_look_point(lat, lon, alt, 0.0, 0.0, 0.0, 0.0, 0.0, steep_tilt, true)
            .expect("intersect");
        let dlat = (steep_off[0] - steep_on[0]).abs();
        let dlon = (steep_off[1] - steep_on[1]).abs();
        assert!(dlat < 1e-8, "steep angle refraction should be negligible: dlat={}", dlat);
        assert!(dlon < 1e-8, "steep angle refraction should be negligible: dlon={}", dlon);
    }

    #[test]
    fn bennett_refraction_values() {
        // At elevation = 0 (horizon), refraction should be positive and significant.
        let r_horizon = bennett_refraction(0.0);
        assert!(r_horizon > 0.0, "refraction at horizon should be positive");
        // Standard atmosphere: ~34 arcminutes ≈ 0.0099 rad at horizon.
        // Bennett's simplified formula gives approximately this.
        assert!(r_horizon > 0.005, "refraction at horizon should be > 5 mrad: {}", r_horizon);
        assert!(r_horizon < 0.02, "refraction at horizon should be < 20 mrad: {}", r_horizon);

        // At 45° elevation, refraction should be very small.
        let r_45 = bennett_refraction(std::f64::consts::FRAC_PI_4);
        assert!(r_45 < 0.001, "refraction at 45° should be < 1 mrad: {}", r_45);
        assert!(r_45 >= 0.0, "refraction should never be negative");
    }
}
