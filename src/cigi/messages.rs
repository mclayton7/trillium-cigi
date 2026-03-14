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

    // ── IgControl::encode ────────────────────────────────────────────────

    #[test]
    fn ig_control_encode_size_and_type() {
        let ig = IgControl::default();
        let enc = ig.encode();
        assert_eq!(enc.len(), 24);
        assert_eq!(enc[0], IgControl::TYPE_ID);
        assert_eq!(enc[1], 24);
    }

    #[test]
    fn ig_control_encode_frame_ctr() {
        let ig = IgControl { frame_ctr: 0x0102_0304, ..Default::default() };
        let enc = ig.encode();
        assert_eq!(&enc[8..12], &0x0102_0304u32.to_le_bytes());
    }

    #[test]
    fn ig_control_encode_db_number() {
        let ig = IgControl { db_number: 42, ..Default::default() };
        let enc = ig.encode();
        assert_eq!(&enc[4..6], &42i16.to_le_bytes());
    }

    #[test]
    fn ig_control_encode_minor_version() {
        let ig = IgControl { minor_version: 3, ..Default::default() };
        let enc = ig.encode();
        assert_eq!(enc[3], 3);
    }

    #[test]
    fn ig_control_encode_flags_byte() {
        let ig = IgControl {
            ig_mode: 2,
            timestamp_valid: true,
            extrapolation_enable: true,
            ..Default::default()
        };
        let enc = ig.encode();
        // bits [7:6] = ig_mode=2, bit1 = extrapolation_enable, bit0 = timestamp_valid
        assert_eq!(enc[2] & 0x03, 0x03); // both flags set
        assert_eq!((enc[2] >> 6) & 0x03, 2); // ig_mode
    }

    // ── EntityControl::encode ────────────────────────────────────────────

    #[test]
    fn entity_control_encode_size_and_type() {
        let ec = EntityControl::default();
        let enc = ec.encode();
        assert_eq!(enc.len(), 48);
        assert_eq!(enc[0], EntityControl::TYPE_ID);
        assert_eq!(enc[1], 48);
    }

    #[test]
    fn entity_control_encode_entity_id() {
        let ec = EntityControl { entity_id: 0x1234, ..Default::default() };
        let enc = ec.encode();
        assert_eq!(&enc[2..4], &0x1234u16.to_le_bytes());
    }

    #[test]
    fn entity_control_encode_roll_f32() {
        let ec = EntityControl { roll: 1.5, ..Default::default() };
        let enc = ec.encode();
        assert_eq!(f32::from_le_bytes([enc[12], enc[13], enc[14], enc[15]]), 1.5);
    }

    #[test]
    fn entity_control_encode_lat_f64() {
        let ec = EntityControl { lat_or_x: 38.897670, ..Default::default() };
        let enc = ec.encode();
        let v = f64::from_le_bytes(enc[24..32].try_into().unwrap());
        assert!((v - 38.897670).abs() < 1e-9);
    }

    #[test]
    fn entity_control_encode_entity_state_bits() {
        let ec = EntityControl { entity_state: 1, attach_state: true, ..Default::default() };
        let enc = ec.encode();
        assert_eq!(enc[4] & 0x03, 1); // entity_state
        assert_ne!(enc[4] & 0x04, 0); // attach_state
    }

    // ── SensorControl geopoint ───────────────────────────────────────────

    #[test]
    fn sensor_control_geopoint_state_4_encoding() {
        let sc = SensorControl { sensor_state: 4, ..Default::default() };
        let enc = sc.encode();
        assert_eq!(enc[4] & 0x07, 4, "byte4 low bits should be 4");
    }

    #[test]
    fn sensor_control_geopoint_response_type_bit() {
        let sc = SensorControl { sensor_state: 4, response_type: true, ..Default::default() };
        let enc = sc.encode();
        assert_ne!(enc[4] & 0x80, 0);
    }

    #[test]
    fn sensor_control_geopoint_polarity_ignored() {
        let sc_pol = SensorControl { sensor_state: 4, polarity: true, ..Default::default() };
        let sc_nopol = SensorControl { sensor_state: 4, polarity: false, ..Default::default() };
        // polarity has no effect in geopoint mode
        assert_eq!(sc_pol.encode()[4], sc_nopol.encode()[4]);
    }

    #[test]
    fn sensor_control_geopoint_roundtrip() {
        let sc = SensorControl { sensor_state: 4, track_mode: 2, gain: 0.3, level: 0.7, ..Default::default() };
        let dec = SensorControl::decode(&sc.encode()).unwrap();
        assert_eq!(dec.sensor_state, 4);
        assert_eq!(dec.track_mode, 2);
    }

    // ── SensorExtendedResponse::decode ───────────────────────────────────

    #[test]
    fn sensor_extended_response_decode_too_short() {
        assert!(SensorExtendedResponse::decode(&[0u8; 47]).is_none());
    }

    #[test]
    fn sensor_extended_response_decode_fields() {
        let mut buf = vec![0u8; 48];
        buf[0] = SensorExtendedResponse::TYPE_ID;
        buf[1] = 48;
        // view_id = 7 at [2..4]
        buf[2..4].copy_from_slice(&7u16.to_le_bytes());
        // sensor_id = 3 at [4]
        buf[4] = 3;
        // sensor_status bits = 1 (tracking) at [5]
        buf[5] = 0x01;
        // gate_x_size at [8..10]
        buf[8..10].copy_from_slice(&20u16.to_le_bytes());
        // gate_x_pos f32 at [12..16]
        buf[12..16].copy_from_slice(&45.0f32.to_le_bytes());
        // entity_lat f64 at [24..32]
        buf[24..32].copy_from_slice(&38.897_f64.to_le_bytes());
        let r = SensorExtendedResponse::decode(&buf).expect("decode");
        assert_eq!(r.view_id, 7);
        assert_eq!(r.sensor_id, 3);
        assert_eq!(r.sensor_status, 1);
        assert_eq!(r.gate_x_size, 20);
        assert!((r.gate_x_pos - 45.0).abs() < 1e-5);
        assert!((r.entity_lat - 38.897).abs() < 1e-6);
    }

    // ── StartOfFrame::decode ─────────────────────────────────────────────

    #[test]
    fn start_of_frame_decode_too_short() {
        assert!(StartOfFrame::decode(&[0u8; 23]).is_none());
    }

    #[test]
    fn start_of_frame_roundtrip_all_fields() {
        let sof = StartOfFrame {
            ig_status: 5,
            ig_mode: 1,
            timestamp_valid: true,
            earth_ref_model: true,
            minor_version: 3,
            db_number: -2,
            ig_frame_ctr: 999,
            timestamp: 1.23456,
            last_host_frame_number: 88,
        };
        let enc = sof.encode();
        let dec = StartOfFrame::decode(&enc).unwrap();
        assert_eq!(dec.ig_status, 5);
        assert_eq!(dec.ig_mode, 1);
        assert!(dec.timestamp_valid);
        assert!(dec.earth_ref_model);
        assert_eq!(dec.minor_version, 3);
        assert_eq!(dec.db_number, -2);
        assert_eq!(dec.ig_frame_ctr, 999);
        assert_eq!(dec.last_host_frame_number, 88);
    }

    #[test]
    fn start_of_frame_timestamp_precision() {
        let sof = StartOfFrame { timestamp: 1.5, ..Default::default() };
        let dec = StartOfFrame::decode(&sof.encode()).unwrap();
        // Stored as 10-µs integer, so precision is 10 µs = 0.00001 s
        assert!((dec.timestamp - 1.5).abs() < 0.00002, "ts={}", dec.timestamp);
    }

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
