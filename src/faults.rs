// Fault injection and diagnostics simulation — Phase 6.1.
//
// Provides:
//   - FaultState  — holds active fault flags and simulated sensor readings
//   - build_diagnostics() → OrionDiagnosticsPacket   (voltages, temps, …)
//   - Periodic console log of diagnostics

use crate::orion::{
    OrionDiagnosticsPacket, OrionFaultComponent, OrionFaultLevel, OrionFaultPacket, OrionFaultType,
};

// ─────────────────────────────────────────── nominal sensor values ──

const NOMINAL_V24: f32 = 24.0;
const NOMINAL_V12: f32 = 12.0;
const NOMINAL_V3V3: f32 = 3.3;
const NOMINAL_CROWN_TEMP: f32 = 45.0; // °C
const NOMINAL_GYRO_TEMP: f32 = 55.0;
const NOMINAL_PAYLOAD_TEMP: f32 = 40.0;
const NOMINAL_HUMIDITY: f32 = 30.0; // %

// ─────────────────────────────────────────── FaultState ──

/// Fault injection state.  All flags default to false (nominal operation).
#[derive(Debug, Clone, Default)]
pub struct FaultState {
    /// GPS/INS signal is lost (position fields unreliable).
    pub gps_loss: bool,
    /// Motor drive fault (slew disabled).
    pub motor_fault: bool,
    /// IMU dropout (angular rates unreliable).
    pub imu_dropout: bool,
    /// Thermal warning (temps elevated).
    pub thermal_warning: bool,

    /// Degraded GPS — adds noise to position fields instead of zeroing them.
    pub degraded_gps: bool,
    /// Standard deviation of Gaussian-like noise added when degraded_gps is active.
    pub gps_noise_std: f64,

    /// Encoder fault — freeze reported pan/tilt at injection-time values.
    pub encoder_fault: bool,
    /// Frozen pan angle captured at encoder fault injection (rad).
    pub frozen_pan: f32,
    /// Frozen tilt angle captured at encoder fault injection (rad).
    pub frozen_tilt: f32,

    /// Laser rangefinder fault — forces `range_source=None` and zero slant range.
    pub laser_fault: bool,

    // Internal: accumulated system time (seconds) for temperature drift model.
    pub(crate) uptime_secs: f32,
}

#[allow(dead_code)]
impl FaultState {
    /// Advance internal time by `dt` seconds.
    pub fn tick(&mut self, dt: f32) {
        self.uptime_secs += dt;
    }

    // ── Injection API ──────────────────────────────────────────────

    pub fn inject_gps_loss(&mut self)    { self.gps_loss = true; }
    pub fn clear_gps_loss(&mut self)     { self.gps_loss = false; }
    pub fn inject_motor_fault(&mut self) { self.motor_fault = true; }
    pub fn clear_motor_fault(&mut self)  { self.motor_fault = false; }
    pub fn inject_imu_dropout(&mut self) { self.imu_dropout = true; }
    pub fn clear_imu_dropout(&mut self)  { self.imu_dropout = false; }
    #[allow(dead_code)]
    pub fn inject_thermal(&mut self)     { self.thermal_warning = true; }
    #[allow(dead_code)]
    pub fn clear_thermal(&mut self)      { self.thermal_warning = false; }

    /// Inject degraded GPS: position fields get Gaussian-like noise instead of
    /// being zeroed.  `noise_std` controls the standard deviation (radians for
    /// lat/lon, metres for altitude).
    pub fn inject_degraded_gps(&mut self, noise_std: f64) {
        self.degraded_gps = true;
        self.gps_noise_std = noise_std;
    }
    pub fn clear_degraded_gps(&mut self) {
        self.degraded_gps = false;
        self.gps_noise_std = 0.0;
    }

    /// Inject encoder fault: freeze reported pan/tilt at the given angles.
    /// The simulator control loop continues normally; only telemetry is
    /// affected.
    pub fn inject_encoder_fault(&mut self, current_pan: f32, current_tilt: f32) {
        self.encoder_fault = true;
        self.frozen_pan = current_pan;
        self.frozen_tilt = current_tilt;
    }
    pub fn clear_encoder_fault(&mut self) {
        self.encoder_fault = false;
        self.frozen_pan = 0.0;
        self.frozen_tilt = 0.0;
    }

    pub fn inject_laser_fault(&mut self) { self.laser_fault = true; }
    pub fn clear_laser_fault(&mut self)  { self.laser_fault = false; }

    pub fn clear_all(&mut self)          {
        self.gps_loss = false;
        self.motor_fault = false;
        self.imu_dropout = false;
        self.thermal_warning = false;
        self.degraded_gps = false;
        self.gps_noise_std = 0.0;
        self.encoder_fault = false;
        self.frozen_pan = 0.0;
        self.frozen_tilt = 0.0;
        self.laser_fault = false;
    }

    // ── Packet builders ──────────────────────────────────────────

    /// Build a simulated `OrionDiagnosticsPacket` reflecting current fault state.
    pub fn build_diagnostics(&self, noise_seed: &mut u32) -> OrionDiagnosticsPacket {
        let temp_drift = (self.uptime_secs / 60.0).min(20.0); // warm up over first minute

        let crown_temp = NOMINAL_CROWN_TEMP + temp_drift
            + if self.thermal_warning { 30.0 } else { 0.0 }
            + lcg_noise_f32(noise_seed) * 0.5;

        let gyro_temp = NOMINAL_GYRO_TEMP + temp_drift * 0.8
            + if self.imu_dropout { 15.0 } else { 0.0 }
            + lcg_noise_f32(noise_seed) * 0.3;

        let payload_temp = NOMINAL_PAYLOAD_TEMP + temp_drift * 0.6
            + lcg_noise_f32(noise_seed) * 0.4;

        // Voltage sag under motor fault
        let v24 = NOMINAL_V24 - if self.motor_fault { 2.0 } else { 0.0 }
            + lcg_noise_f32(noise_seed) * 0.05;
        let v12 = NOMINAL_V12 + lcg_noise_f32(noise_seed) * 0.02;
        let v3v3 = NOMINAL_V3V3 + lcg_noise_f32(noise_seed) * 0.01;

        OrionDiagnosticsPacket {
            voltage24: v24,
            voltage12: v12,
            voltage3v3: v3v3,
            current24: 1.2 + if self.motor_fault { 2.5 } else { 0.0 },
            current12: 0.8,
            current3v3: 0.4,
            crown_temp,
            sla_temp: crown_temp - 5.0,
            gyro_temp,
            voltage24var: lcg_noise_f32(noise_seed).abs() * 0.002,
            voltage12var: lcg_noise_f32(noise_seed).abs() * 0.001,
            voltage3v3var: lcg_noise_f32(noise_seed).abs() * 0.0005,
            current24var: 0.01,
            current12var: 0.005,
            current3v3var: 0.002,
            payload_temp,
            payload_humidity: NOMINAL_HUMIDITY + lcg_noise_f32(noise_seed) * 2.0,
            current_laser: 0.0,
        }
    }

    /// Build `OrionFaultPacket`s for all active faults.
    pub fn build_fault_packets(&self) -> Vec<OrionFaultPacket> {
        let mut pkts = Vec::new();
        if self.motor_fault {
            pkts.push(OrionFaultPacket {
                type_: OrionFaultType::FaultTypeVelocityLimitExceeded,
                level: OrionFaultLevel::FaultLevelError,
                component: OrionFaultComponent::FaultComponentPanAxis,
                ..Default::default()
            });
        }
        if self.gps_loss {
            // NOTE: The Orion protocol XML defines no GPS-specific fault type.
            // FaultTypeNone + FaultLevelWarning is a convention for GPS loss.
            // Update if a GPS fault type is added to OrionPublicProtocol.xml.
            pkts.push(OrionFaultPacket {
                type_: OrionFaultType::FaultTypeNone,
                level: OrionFaultLevel::FaultLevelWarning,
                component: OrionFaultComponent::FaultComponentPayloadGyros,
                ..Default::default()
            });
        }
        if self.imu_dropout {
            pkts.push(OrionFaultPacket {
                type_: OrionFaultType::FaultTypeInvalidGyroCalibration,
                level: OrionFaultLevel::FaultLevelError,
                component: OrionFaultComponent::FaultComponentPayloadGyros,
                ..Default::default()
            });
        }
        pkts
    }

    /// Print a one-line diagnostic summary (called at ~1 Hz).
    pub fn log_diagnostics(&self, diag: &OrionDiagnosticsPacket) {
        println!(
            "[DIAG] V24={:.2}V  V12={:.2}V  V3V3={:.2}V  \
             Crown={:.1}°C  Gyro={:.1}°C  Payload={:.1}°C  \
             Faults: GPS={} Motor={} IMU={}",
            diag.voltage24, diag.voltage12, diag.voltage3v3,
            diag.crown_temp, diag.gyro_temp, diag.payload_temp,
            self.gps_loss as u8, self.motor_fault as u8, self.imu_dropout as u8,
        );
    }
}

// ─────────────────────────────────────────── LCG noise ──

/// Simple LCG pseudo-random noise in [−1, 1].
pub fn lcg_noise_f32(seed: &mut u32) -> f32 {
    *seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
    ((*seed >> 16) as f32 / 32768.0) - 1.0
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── lcg_noise_f32 ─────────────────────────────────────────────────────

    #[test]
    fn lcg_noise_range() {
        let mut seed = 0xABCD_1234u32;
        for _ in 0..1000 {
            let v = lcg_noise_f32(&mut seed);
            assert!(v >= -1.0 && v <= 1.0, "out of range: {v}");
        }
    }

    #[test]
    fn lcg_noise_deterministic() {
        let mut s1 = 42u32;
        let mut s2 = 42u32;
        assert_eq!(lcg_noise_f32(&mut s1).to_bits(), lcg_noise_f32(&mut s2).to_bits());
        assert_eq!(s1, s2);
    }

    #[test]
    fn lcg_noise_advances_seed() {
        let mut seed = 1u32;
        let before = seed;
        lcg_noise_f32(&mut seed);
        assert_ne!(seed, before);
    }

    // ── FaultState::tick ──────────────────────────────────────────────────

    #[test]
    fn tick_accumulates_uptime() {
        let mut fs = FaultState::default();
        assert_eq!(fs.uptime_secs, 0.0);
        fs.tick(0.5);
        assert!((fs.uptime_secs - 0.5).abs() < 1e-6);
        fs.tick(1.5);
        assert!((fs.uptime_secs - 2.0).abs() < 1e-6);
    }

    // ── inject / clear API ────────────────────────────────────────────────

    #[test]
    fn inject_and_clear_gps_loss() {
        let mut fs = FaultState::default();
        assert!(!fs.gps_loss);
        fs.inject_gps_loss();
        assert!(fs.gps_loss);
        fs.clear_gps_loss();
        assert!(!fs.gps_loss);
    }

    #[test]
    fn inject_and_clear_motor_fault() {
        let mut fs = FaultState::default();
        fs.inject_motor_fault();
        assert!(fs.motor_fault);
        fs.clear_motor_fault();
        assert!(!fs.motor_fault);
    }

    #[test]
    fn inject_and_clear_imu_dropout() {
        let mut fs = FaultState::default();
        fs.inject_imu_dropout();
        assert!(fs.imu_dropout);
        fs.clear_imu_dropout();
        assert!(!fs.imu_dropout);
    }

    #[test]
    fn inject_and_clear_thermal() {
        let mut fs = FaultState::default();
        fs.inject_thermal();
        assert!(fs.thermal_warning);
        fs.clear_thermal();
        assert!(!fs.thermal_warning);
    }

    #[test]
    fn inject_and_clear_degraded_gps() {
        let mut fs = FaultState::default();
        assert!(!fs.degraded_gps);
        fs.inject_degraded_gps(0.001);
        assert!(fs.degraded_gps);
        assert!((fs.gps_noise_std - 0.001).abs() < 1e-12);
        fs.clear_degraded_gps();
        assert!(!fs.degraded_gps);
        assert_eq!(fs.gps_noise_std, 0.0);
    }

    #[test]
    fn inject_and_clear_encoder_fault() {
        let mut fs = FaultState::default();
        assert!(!fs.encoder_fault);
        fs.inject_encoder_fault(0.5, -0.3);
        assert!(fs.encoder_fault);
        assert!((fs.frozen_pan - 0.5).abs() < 1e-6);
        assert!((fs.frozen_tilt - (-0.3)).abs() < 1e-6);
        fs.clear_encoder_fault();
        assert!(!fs.encoder_fault);
        assert_eq!(fs.frozen_pan, 0.0);
        assert_eq!(fs.frozen_tilt, 0.0);
    }

    #[test]
    fn clear_all_resets_all_flags() {
        let mut fs = FaultState::default();
        fs.inject_gps_loss();
        fs.inject_motor_fault();
        fs.inject_imu_dropout();
        fs.inject_thermal();
        fs.inject_degraded_gps(0.01);
        fs.inject_encoder_fault(1.0, 2.0);
        fs.clear_all();
        assert!(!fs.gps_loss);
        assert!(!fs.motor_fault);
        assert!(!fs.imu_dropout);
        assert!(!fs.thermal_warning);
        assert!(!fs.degraded_gps);
        assert_eq!(fs.gps_noise_std, 0.0);
        assert!(!fs.encoder_fault);
        assert_eq!(fs.frozen_pan, 0.0);
        assert_eq!(fs.frozen_tilt, 0.0);
    }

    // ── build_fault_packets ───────────────────────────────────────────────

    #[test]
    fn build_fault_packets_empty_when_nominal() {
        let fs = FaultState::default();
        assert!(fs.build_fault_packets().is_empty());
    }

    #[test]
    fn build_fault_packets_motor_fault() {
        let mut fs = FaultState::default();
        fs.inject_motor_fault();
        let pkts = fs.build_fault_packets();
        assert_eq!(pkts.len(), 1);
        assert!(matches!(pkts[0].type_, OrionFaultType::FaultTypeVelocityLimitExceeded));
        assert!(matches!(pkts[0].level, OrionFaultLevel::FaultLevelError));
    }

    #[test]
    fn build_fault_packets_gps_loss() {
        let mut fs = FaultState::default();
        fs.inject_gps_loss();
        let pkts = fs.build_fault_packets();
        assert_eq!(pkts.len(), 1);
        assert!(matches!(pkts[0].type_, OrionFaultType::FaultTypeNone));
        assert!(matches!(pkts[0].level, OrionFaultLevel::FaultLevelWarning));
    }

    #[test]
    fn build_fault_packets_imu_dropout() {
        let mut fs = FaultState::default();
        fs.inject_imu_dropout();
        let pkts = fs.build_fault_packets();
        assert_eq!(pkts.len(), 1);
        assert!(matches!(pkts[0].type_, OrionFaultType::FaultTypeInvalidGyroCalibration));
        assert!(matches!(pkts[0].level, OrionFaultLevel::FaultLevelError));
        assert!(matches!(pkts[0].component, OrionFaultComponent::FaultComponentPayloadGyros));
    }

    #[test]
    fn build_fault_packets_multiple_faults() {
        let mut fs = FaultState::default();
        fs.inject_motor_fault();
        fs.inject_gps_loss();
        fs.inject_imu_dropout();
        let pkts = fs.build_fault_packets();
        assert_eq!(pkts.len(), 3, "expected 3 fault packets, got {}", pkts.len());
    }

    // ── build_diagnostics ─────────────────────────────────────────────────

    #[test]
    fn build_diagnostics_nominal_voltages() {
        let fs = FaultState::default();
        let mut seed = 0u32;
        let d = fs.build_diagnostics(&mut seed);
        // Voltages should be near nominal (noise is tiny)
        assert!((d.voltage24 - 24.0).abs() < 0.5, "v24={}", d.voltage24);
        assert!((d.voltage12 - 12.0).abs() < 0.1, "v12={}", d.voltage12);
        assert!((d.voltage3v3 - 3.3).abs() < 0.1, "v3v3={}", d.voltage3v3);
    }

    #[test]
    fn build_diagnostics_motor_fault_voltage_sag() {
        let mut fs = FaultState::default();
        let mut seed = 42u32;
        let nominal = fs.build_diagnostics(&mut seed).voltage24;
        fs.inject_motor_fault();
        let mut seed2 = 42u32;
        let sagged = fs.build_diagnostics(&mut seed2).voltage24;
        assert!(sagged < nominal - 1.0, "expected voltage sag: nominal={nominal} sagged={sagged}");
    }

    #[test]
    fn build_diagnostics_thermal_crown_temp_rise() {
        let mut fs = FaultState::default();
        let mut seed = 0u32;
        let baseline = fs.build_diagnostics(&mut seed).crown_temp;
        fs.inject_thermal();
        let mut seed2 = 0u32;
        let hot = fs.build_diagnostics(&mut seed2).crown_temp;
        assert!(hot > baseline + 20.0, "expected temp rise: baseline={baseline} hot={hot}");
    }

    #[test]
    fn build_diagnostics_imu_dropout_gyro_temp_rise() {
        let mut fs = FaultState::default();
        let mut seed = 0u32;
        let baseline = fs.build_diagnostics(&mut seed).gyro_temp;
        fs.inject_imu_dropout();
        let mut seed2 = 0u32;
        let hot = fs.build_diagnostics(&mut seed2).gyro_temp;
        assert!(hot > baseline + 10.0, "expected gyro temp rise: baseline={baseline} hot={hot}");
    }

    #[test]
    fn build_diagnostics_warmup_cap() {
        // After a very long uptime, temperature drift should be capped at 20°C
        let mut fs = FaultState::default();
        fs.uptime_secs = 10_000.0; // far past the 60 s warmup
        let mut seed = 0u32;
        let d = fs.build_diagnostics(&mut seed);
        // crown_temp = 45 + 20 (cap) + noise*0.5 → should be ≤ 66
        assert!(d.crown_temp < 67.0, "crown_temp={}", d.crown_temp);
    }
}
