// CIGI v3.3 message definitions (hand-written per ICD).
// All multi-byte fields are little-endian as per the CIGI standard.

// ─────────────────────────────────────────── helpers ──

fn read_u8(buf: &[u8], off: usize) -> Option<u8> {
    buf.get(off).copied()
}
fn read_u16_le(buf: &[u8], off: usize) -> Option<u16> {
    Some(u16::from_le_bytes([*buf.get(off)?, *buf.get(off + 1)?]))
}
fn read_u32_le(buf: &[u8], off: usize) -> Option<u32> {
    Some(u32::from_le_bytes([
        *buf.get(off)?,
        *buf.get(off + 1)?,
        *buf.get(off + 2)?,
        *buf.get(off + 3)?,
    ]))
}
fn read_i16_le(buf: &[u8], off: usize) -> Option<i16> {
    Some(i16::from_le_bytes([*buf.get(off)?, *buf.get(off + 1)?]))
}
fn read_f32_le(buf: &[u8], off: usize) -> Option<f32> {
    Some(f32::from_le_bytes([
        *buf.get(off)?,
        *buf.get(off + 1)?,
        *buf.get(off + 2)?,
        *buf.get(off + 3)?,
    ]))
}
fn read_f64_le(buf: &[u8], off: usize) -> Option<f64> {
    Some(f64::from_le_bytes([
        *buf.get(off)?,
        *buf.get(off + 1)?,
        *buf.get(off + 2)?,
        *buf.get(off + 3)?,
        *buf.get(off + 4)?,
        *buf.get(off + 5)?,
        *buf.get(off + 6)?,
        *buf.get(off + 7)?,
    ]))
}

// ══════════════════════════════════════════════════════════
//  Host → IG
// ══════════════════════════════════════════════════════════

/// CIGI IG Control (type 1, 24 bytes).
#[allow(dead_code)]
#[derive(Debug, Clone, Default)]
pub struct IgControl {
    pub ig_mode: u8,
    pub timestamp_valid: bool,
    pub extrapolation_enable: bool,
    pub minor_version: u8,
    pub db_number: i16,
    pub last_rcvd_ig_frame_ctr: u16,
    pub frame_ctr: u32,
    pub timestamp: f64,
}

impl IgControl {
    pub const TYPE_ID: u8 = 1;

    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < 24 { return None; }
        let flags = read_u8(buf, 2)?;
        Some(Self {
            ig_mode: (flags >> 6) & 0x03,
            timestamp_valid: flags & 0x01 != 0,
            extrapolation_enable: flags & 0x02 != 0,
            minor_version: read_u8(buf, 3)?,
            db_number: read_i16_le(buf, 4)?,
            last_rcvd_ig_frame_ctr: read_u16_le(buf, 6)?,
            frame_ctr: read_u32_le(buf, 8)?,
            timestamp: read_f64_le(buf, 12)?,
        })
    }
}

/// CIGI Entity Control (type 2, 48 bytes).
#[allow(dead_code)]
#[derive(Debug, Clone, Default)]
pub struct EntityControl {
    pub entity_id: u16,
    pub entity_state: u8,
    pub attach_state: bool,
    pub collision_detect: bool,
    pub inherit_alpha: bool,
    pub ground_occlude_clamped: bool,
    pub animation_dir: bool,
    pub animation_mode: u8,
    pub animation_state: u8,
    pub alpha: u8,
    pub entity_type: u16,
    pub parent_id: u16,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub lat_or_x: f64,
    pub lon_or_y: f64,
    pub alt_or_z: f64,
}

impl EntityControl {
    pub const TYPE_ID: u8 = 2;

    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < 48 { return None; }
        let flags = read_u8(buf, 4)?;
        let anim = read_u8(buf, 5)?;
        Some(Self {
            entity_id: read_u16_le(buf, 2)?,
            entity_state: flags & 0x03,
            attach_state: flags & 0x04 != 0,
            collision_detect: flags & 0x08 != 0,
            inherit_alpha: flags & 0x10 != 0,
            ground_occlude_clamped: flags & 0x20 != 0,
            animation_dir: flags & 0x40 != 0,
            animation_mode: anim & 0x03,
            animation_state: (anim >> 2) & 0x03,
            alpha: read_u8(buf, 6)?,
            entity_type: read_u16_le(buf, 8)?,
            parent_id: read_u16_le(buf, 10)?,
            roll: read_f32_le(buf, 12)?,
            pitch: read_f32_le(buf, 16)?,
            yaw: read_f32_le(buf, 20)?,
            lat_or_x: read_f64_le(buf, 24)?,
            lon_or_y: read_f64_le(buf, 32)?,
            alt_or_z: read_f64_le(buf, 40)?,
        })
    }
}

/// CIGI Sensor Control (type 17, 24 bytes).
#[allow(dead_code)]
#[derive(Debug, Clone, Default)]
pub struct SensorControl {
    pub view_id: u8,
    pub sensor_id: u8,
    /// 0=Inactive, 1=Active, 2=Tracking
    pub sensor_state: u8,
    pub polarity: bool,
    pub line_of_sight_enable: bool,
    pub track_mode: u8,
    pub response_type: bool,
    pub auto_gain: bool,
    pub track_polarity: bool,
    pub gain: f32,
    pub level: f32,
    pub ac_coupling: f32,
    pub noise: f32,
}

impl SensorControl {
    pub const TYPE_ID: u8 = 17;

    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < 24 { return None; }
        let f4 = read_u8(buf, 4)?;
        let f5 = read_u8(buf, 5)?;
        Some(Self {
            view_id: read_u8(buf, 2)?,
            sensor_id: read_u8(buf, 3)?,
            // Standard CIGI encodes sensor_state in bits 0-1 (values 0-3).
            // The simulator's geopoint extension (value 4) is host-internal and
            // never arrives from a real GCS, so this mask is intentionally narrow.
            sensor_state: f4 & 0x03,
            polarity: f4 & 0x04 != 0,
            line_of_sight_enable: f4 & 0x08 != 0,
            track_mode: (f4 >> 4) & 0x07,
            response_type: f4 & 0x80 != 0,
            auto_gain: f5 & 0x01 != 0,
            track_polarity: f5 & 0x02 != 0,
            gain: read_f32_le(buf, 6)?,
            level: read_f32_le(buf, 10)?,
            ac_coupling: read_f32_le(buf, 14)?,
            noise: read_f32_le(buf, 18)?,
        })
    }
}

// ══════════════════════════════════════════════════════════
//  IG → Host
// ══════════════════════════════════════════════════════════

/// CIGI Start Of Frame (type 101, 24 bytes).
#[derive(Debug, Clone, Default)]
pub struct StartOfFrame {
    pub ig_status: u8,
    pub ig_mode: u8,
    pub timestamp_valid: bool,
    pub earth_ref_model: bool,
    pub minor_version: u8,
    pub db_number: i16,
    pub ig_frame_ctr: u32,
    pub timestamp: f64,
    pub last_host_frame_number: u32,
}

impl StartOfFrame {
    pub const TYPE_ID: u8 = 101;
    pub const SIZE: u8 = 24;

    pub fn encode(&self) -> Vec<u8> {
        let mut out = vec![0u8; 24];
        out[0] = Self::TYPE_ID;
        out[1] = Self::SIZE;
        out[2] = 3; // Major Version
        out[3] = self.db_number as i8 as u8;
        out[4] = self.ig_status;
        out[5] = ((self.minor_version & 0x0F) << 4)
            | if self.earth_ref_model { 0x08 } else { 0 }
            | if self.timestamp_valid { 0x04 } else { 0 }
            | (self.ig_mode & 0x03);
        out[6..8].copy_from_slice(&0x8000u16.to_le_bytes());
        out[8..12].copy_from_slice(&self.ig_frame_ctr.to_le_bytes());
        let ts_10us = (self.timestamp * 100_000.0) as u64 as u32; // intentional wrap every ~12 h
        out[12..16].copy_from_slice(&ts_10us.to_le_bytes());
        out[16..20].copy_from_slice(&self.last_host_frame_number.to_le_bytes());
        // bytes 20-23: reserved = 0
        out
    }

}



/// CIGI Sensor Extended Response (type 107, 48 bytes).
#[derive(Debug, Clone, Default)]
pub struct SensorExtendedResponse {
    pub view_id: u16,
    pub sensor_id: u8,
    pub sensor_status: u8,
    pub gate_x_size: u16,
    pub gate_y_size: u16,
    pub gate_x_pos: f32,
    pub gate_y_pos: f32,
    pub frame_ctr: u32,
    pub entity_id_valid: bool,
    pub entity_id: u16,
    pub entity_lat: f64,
    pub entity_lon: f64,
    pub entity_alt: f64,
}

impl SensorExtendedResponse {
    pub const TYPE_ID: u8 = 107;
    pub const SIZE: u8 = 48;

    pub fn encode(&self) -> Vec<u8> {
        let mut out = vec![0u8; 48];
        out[0] = Self::TYPE_ID;
        out[1] = Self::SIZE;
        out[2..4].copy_from_slice(&self.view_id.to_le_bytes());
        out[4] = self.sensor_id;
        out[5] = (self.sensor_status & 0x03)
            | if self.entity_id_valid { 0x04 } else { 0 };
        out[6..8].copy_from_slice(&self.entity_id.to_le_bytes());
        out[8..10].copy_from_slice(&self.gate_x_size.to_le_bytes());
        out[10..12].copy_from_slice(&self.gate_y_size.to_le_bytes());
        out[12..16].copy_from_slice(&self.gate_x_pos.to_le_bytes());
        out[16..20].copy_from_slice(&self.gate_y_pos.to_le_bytes());
        out[20..24].copy_from_slice(&self.frame_ctr.to_le_bytes());
        out[24..32].copy_from_slice(&self.entity_lat.to_le_bytes());
        out[32..40].copy_from_slice(&self.entity_lon.to_le_bytes());
        out[40..48].copy_from_slice(&self.entity_alt.to_le_bytes());
        out
    }

}

#[cfg(test)]
impl SensorControl {
    pub fn encode(&self) -> Vec<u8> {
        let mut out = vec![0u8; 24];
        out[0] = Self::TYPE_ID;
        out[1] = 24;
        out[2] = self.view_id;
        out[3] = self.sensor_id;
        out[4] = (self.sensor_state & 0x03)
            | if self.polarity { 0x04 } else { 0 }
            | if self.line_of_sight_enable { 0x08 } else { 0 }
            | ((self.track_mode & 0x07) << 4)
            | if self.response_type { 0x80 } else { 0 };
        out[5] = if self.auto_gain { 0x01 } else { 0 }
            | if self.track_polarity { 0x02 } else { 0 };
        out[6..10].copy_from_slice(&self.gain.to_le_bytes());
        out[10..14].copy_from_slice(&self.level.to_le_bytes());
        out[14..18].copy_from_slice(&self.ac_coupling.to_le_bytes());
        out[18..22].copy_from_slice(&self.noise.to_le_bytes());
        out
    }
}

#[cfg(test)]
impl StartOfFrame {
    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < 24 { return None; }
        let flags = buf[5];
        Some(Self {
            ig_status: buf[4],
            ig_mode: flags & 0x03,
            timestamp_valid: flags & 0x04 != 0,
            earth_ref_model: flags & 0x08 != 0,
            minor_version: (flags >> 4) & 0x0F,
            db_number: buf[3] as i8 as i16,
            ig_frame_ctr: u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
            timestamp: u32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]) as f64 / 100_000.0,
            last_host_frame_number: u32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sensor_control_roundtrip() {
        let sc = SensorControl {
            view_id: 1,
            sensor_id: 2,
            sensor_state: 1,
            polarity: true,
            line_of_sight_enable: false,
            track_mode: 3,
            response_type: true,
            auto_gain: false,
            track_polarity: true,
            gain: 0.5,
            level: 0.75,
            ac_coupling: 0.1,
            noise: 0.0,
        };
        let enc = sc.encode();
        assert_eq!(enc.len(), 24);
        let dec = SensorControl::decode(&enc).expect("decode");
        assert_eq!(dec.view_id, sc.view_id);
        assert_eq!(dec.sensor_id, sc.sensor_id);
        assert_eq!(dec.sensor_state, sc.sensor_state);
        assert_eq!(dec.polarity, sc.polarity);
        assert!((dec.gain - sc.gain).abs() < 1e-6);
    }

    #[test]
    fn start_of_frame_roundtrip() {
        let sof = StartOfFrame {
            ig_status: 0,
            ig_mode: 2,
            timestamp_valid: true,
            earth_ref_model: false,
            minor_version: 3,
            db_number: -1,
            ig_frame_ctr: 42,
            timestamp: 1.5,
            last_host_frame_number: 7,
        };
        let enc = sof.encode();
        assert_eq!(enc.len(), 24);
        let dec = StartOfFrame::decode(&enc).expect("decode");
        assert_eq!(dec.ig_frame_ctr, 42);
        assert_eq!(dec.last_host_frame_number, 7);
    }
}
