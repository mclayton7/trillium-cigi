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
/// `pan` is the azimuth in the NED frame (0 = north, + clockwise), radians.
/// `tilt` is the depression angle (0 = horizontal, + downward), radians.
/// Platform attitude (roll/pitch/yaw) is included for the inertial→NED mapping.
/// For a fully stabilised gimbal the pan/tilt are already inertial, so we add
/// platform yaw only to convert from "nose-relative" to NED azimuth.
pub fn los_ecef(
    lat: f64,
    lon: f64,
    platform_yaw: f32,
    pan: f32,
    tilt: f32,
) -> [f32; 3] {
    // Inertial NED azimuth = gimbal pan + platform yaw heading
    let az = (pan + platform_yaw) as f64;
    let el = tilt as f64; // positive = depression (downward)

    let n = el.cos() * az.cos();
    let e = el.cos() * az.sin();
    let d = el.sin(); // positive down

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
pub fn compute_look_point(
    pos_lat: f64,
    pos_lon: f64,
    pos_alt: f64,
    platform_yaw: f32,
    pan: f32,
    tilt: f32,
) -> Option<[f64; 3]> {
    if pos_alt < 1.0 {
        return None; // on the ground, no look-point
    }
    let origin = geodetic_to_ecef(pos_lat, pos_lon, pos_alt);
    let dir_f32 = los_ecef(pos_lat, pos_lon, platform_yaw, pan, tilt);
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
        let result = compute_look_point(lat, lon, alt, 0.0, 0.0, std::f32::consts::FRAC_PI_2);
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
        if let Some([tl, to, ta]) = compute_look_point(pos_lat, pos_lon, pos_alt, 0.0, pan0, tilt0) {
            let (pan1, tilt1) = inverse_geopoint(pos_lat, pos_lon, pos_alt, 0.0, tl, to, ta);
            assert!((pan1 - pan0).abs() < 0.01, "pan inverse: {} vs {}", pan1, pan0);
            assert!((tilt1 - tilt0).abs() < 0.01, "tilt inverse: {} vs {}", tilt1, tilt0);
        }
    }
}
