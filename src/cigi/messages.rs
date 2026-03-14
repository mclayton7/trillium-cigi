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

    pub fn encode(&self) -> Vec<u8> {
        let mut out = vec![0u8; 24];
        out[0] = Self::TYPE_ID;
        out[1] = 24;
        out[2] = ((self.ig_mode & 0x03) << 6)
            | if self.extrapolation_enable { 0x02 } else { 0 }
            | if self.timestamp_valid { 0x01 } else { 0 };
        out[3] = self.minor_version;
        out[4..6].copy_from_slice(&self.db_number.to_le_bytes());
        out[6..8].copy_from_slice(&self.last_rcvd_ig_frame_ctr.to_le_bytes());
        out[8..12].copy_from_slice(&self.frame_ctr.to_le_bytes());
        out[12..20].copy_from_slice(&self.timestamp.to_le_bytes());
        out
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

    pub fn encode(&self) -> Vec<u8> {
        let mut out = vec![0u8; 48];
        out[0] = Self::TYPE_ID;
        out[1] = 48;
        out[2..4].copy_from_slice(&self.entity_id.to_le_bytes());
        out[4] = (self.entity_state & 0x03)
            | if self.attach_state { 0x04 } else { 0 }
            | if self.collision_detect { 0x08 } else { 0 }
            | if self.inherit_alpha { 0x10 } else { 0 }
            | if self.ground_occlude_clamped { 0x20 } else { 0 }
            | if self.animation_dir { 0x40 } else { 0 };
        out[5] = (self.animation_mode & 0x03) | ((self.animation_state & 0x03) << 2);
        out[6] = self.alpha;
        out[8..10].copy_from_slice(&self.entity_type.to_le_bytes());
        out[10..12].copy_from_slice(&self.parent_id.to_le_bytes());
        out[12..16].copy_from_slice(&self.roll.to_le_bytes());
        out[16..20].copy_from_slice(&self.pitch.to_le_bytes());
        out[20..24].copy_from_slice(&self.yaw.to_le_bytes());
        out[24..32].copy_from_slice(&self.lat_or_x.to_le_bytes());
        out[32..40].copy_from_slice(&self.lon_or_y.to_le_bytes());
        out[40..48].copy_from_slice(&self.alt_or_z.to_le_bytes());
        out
    }
}

/// CIGI Sensor Control (type 17, 24 bytes).
#[allow(dead_code)]
#[derive(Debug, Clone, Default)]
pub struct SensorControl {
    pub view_id: u8,
    pub sensor_id: u8,
    /// 0=Inactive, 1=Active, 2=Tracking, 4=Geopoint (proprietary extension)
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

    /// Encode for transmission to a CIGI scene generator.
    ///
    /// sensor_state values 0–3 follow the CIGI v3.3 standard.
    /// sensor_state = 4 is a proprietary extension for geopoint mode; it uses
    /// bit 2 of byte 4, which overlaps with the polarity flag in the standard.
    /// When sensor_state = 4, polarity is meaningless and omitted.
    pub fn encode(&self) -> Vec<u8> {
        let mut out = vec![0u8; 24];
        out[0] = Self::TYPE_ID;
        out[1] = 24;
        out[2] = self.view_id;
        out[3] = self.sensor_id;
        if self.sensor_state == 4 {
            // Geopoint extension: raw value 4 (bit 2 set). Polarity/LOS not used.
            out[4] = 0x04
                | ((self.track_mode & 0x07) << 4)
                | if self.response_type { 0x80 } else { 0 };
        } else {
            out[4] = (self.sensor_state & 0x03)
                | if self.polarity { 0x04 } else { 0 }
                | if self.line_of_sight_enable { 0x08 } else { 0 }
                | ((self.track_mode & 0x07) << 4)
                | if self.response_type { 0x80 } else { 0 };
        }
        out[5] = if self.auto_gain { 0x01 } else { 0 }
            | if self.track_polarity { 0x02 } else { 0 };
        out[6..10].copy_from_slice(&self.gain.to_le_bytes());
        out[10..14].copy_from_slice(&self.level.to_le_bytes());
        out[14..18].copy_from_slice(&self.ac_coupling.to_le_bytes());
        out[18..22].copy_from_slice(&self.noise.to_le_bytes());
        out
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

    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < 48 { return None; }
        let f5 = read_u8(buf, 5)?;
        Some(Self {
            view_id: read_u16_le(buf, 2)?,
            sensor_id: read_u8(buf, 4)?,
            sensor_status: f5 & 0x03,
            entity_id_valid: f5 & 0x04 != 0,
            entity_id: read_u16_le(buf, 6)?,
            gate_x_size: read_u16_le(buf, 8)?,
            gate_y_size: read_u16_le(buf, 10)?,
            gate_x_pos: read_f32_le(buf, 12)?,
            gate_y_pos: read_f32_le(buf, 16)?,
            frame_ctr: read_u32_le(buf, 20)?,
            entity_lat: read_f64_le(buf, 24)?,
            entity_lon: read_f64_le(buf, 32)?,
            entity_alt: read_f64_le(buf, 40)?,
        })
    }
}

#[cfg(test)]
impl SensorControl {
    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < 24 { return None; }
        let f4 = read_u8(buf, 4)?;
        let f5 = read_u8(buf, 5)?;
        // Detect geopoint extension (sensor_state=4): bits 0-2 = 0b100.
        let raw3 = f4 & 0x07;
        let (sensor_state, polarity) = if raw3 == 4 {
            (4u8, false)
        } else {
            (f4 & 0x03, f4 & 0x04 != 0)
        };
        Some(Self {
            view_id: read_u8(buf, 2)?,
            sensor_id: read_u8(buf, 3)?,
            sensor_state,
            polarity,
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

#[cfg(test)]
impl StartOfFrame {
    pub fn encode(&self) -> Vec<u8> {
        const SIZE: u8 = 24;
        let mut out = vec![0u8; 24];
        out[0] = Self::TYPE_ID;
        out[1] = SIZE;
        out[2] = 3; // Major Version
        out[3] = self.db_number as i8 as u8;
        out[4] = self.ig_status;
        out[5] = ((self.minor_version & 0x0F) << 4)
            | if self.earth_ref_model { 0x08 } else { 0 }
            | if self.timestamp_valid { 0x04 } else { 0 }
            | (self.ig_mode & 0x03);
        out[6..8].copy_from_slice(&0x8000u16.to_le_bytes());
        out[8..12].copy_from_slice(&self.ig_frame_ctr.to_le_bytes());
        let ts_10us = (self.timestamp * 100_000.0) as u64 as u32;
        out[12..16].copy_from_slice(&ts_10us.to_le_bytes());
        out[16..20].copy_from_slice(&self.last_host_frame_number.to_le_bytes());
        out
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
