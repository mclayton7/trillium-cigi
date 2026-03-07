// Fault injection and diagnostics simulation — Phase 6.1.
//
// Provides:
//   - FaultState  — holds active fault flags and simulated sensor readings
//   - build_diagnostics() → OrionDiagnosticsPacket   (voltages, temps, …)
//   - Periodic console log of diagnostics

use crate::orion::{OrionDiagnosticsPacket, OrionFaultLevel, OrionFaultPacket, OrionFaultType};

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

    // Internal: accumulated system time (seconds) for temperature drift model.
    pub(crate) uptime_secs: f32,
}

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
    pub fn inject_thermal(&mut self)     { self.thermal_warning = true; }
    pub fn clear_thermal(&mut self)      { self.thermal_warning = false; }
    pub fn clear_all(&mut self)          {
        self.gps_loss = false;
        self.motor_fault = false;
        self.imu_dropout = false;
        self.thermal_warning = false;
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

    /// Build an `OrionFaultPacket` if any active fault warrants one.
    pub fn build_fault_packet(&self) -> Option<OrionFaultPacket> {
        if self.motor_fault {
            return Some(OrionFaultPacket {
                type_: OrionFaultType::FaultTypeVelocityLimitExceeded,
                level: OrionFaultLevel::FaultLevelError,
                ..Default::default()
            });
        }
        None
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
