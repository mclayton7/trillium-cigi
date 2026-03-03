// Orion wire-protocol framing.
//
// Packet format:
//   [0xD0, 0x0D, id, len, ...data(len bytes)..., ck0, ck1]
//
// Checksum is modified Fletcher-16 (mod 251, initial value from byte 0).

/// Compute the Orion modified Fletcher-16 checksum.
///
/// Covers bytes 0 through (4 + payload_len - 1), i.e. sync + id + len + data.
pub fn checksum(bytes: &[u8]) -> (u8, u8) {
    if bytes.is_empty() {
        return (1, 1);
    }
    let mut a = ((bytes[0] as u16) + 1) % 251;
    let mut b = a;
    for &byte in &bytes[1..] {
        a = (a + byte as u16) % 251;
        b = (b + a) % 251;
    }
    (a as u8, b as u8)
}

/// Wrap a payload in the Orion frame: [0xD0, 0x0D, id, len, ...data..., ck0, ck1].
pub fn frame(id: u8, payload: &[u8]) -> Vec<u8> {
    let len = payload.len() as u8;
    let mut pkt = Vec::with_capacity(6 + payload.len());
    pkt.push(0xD0);
    pkt.push(0x0D);
    pkt.push(id);
    pkt.push(len);
    pkt.extend_from_slice(payload);
    let (ck0, ck1) = checksum(&pkt[..4 + payload.len()]);
    pkt.push(ck0);
    pkt.push(ck1);
    pkt
}

/// Scan `buf` for the first valid Orion packet.
///
/// Returns `Some((packet_id, payload_slice, total_bytes_consumed))` on success.
pub fn parse(buf: &[u8]) -> Option<(u8, &[u8], usize)> {
    let mut start = 0;
    while start + 4 <= buf.len() {
        if buf[start] == 0xD0 && buf[start + 1] == 0x0D {
            let id = buf[start + 2];
            let len = buf[start + 3] as usize;
            let total = 4 + len + 2; // header + data + 2 checksum bytes
            if start + total > buf.len() {
                break; // not enough data yet
            }
            let data = &buf[start + 4..start + 4 + len];
            let (ck0, ck1) = checksum(&buf[start..start + 4 + len]);
            let ck0_rcvd = buf[start + 4 + len];
            let ck1_rcvd = buf[start + 4 + len + 1];
            if ck0 == ck0_rcvd && ck1 == ck1_rcvd {
                return Some((id, data, start + total));
            }
        }
        start += 1;
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::orion::{OrionCmdPacket, OrionMode};

    #[test]
    fn frame_parse_roundtrip() {
        let payload = [0x01u8, 0x02, 0x03, 0x04];
        let framed = frame(0x42, &payload);
        let (id, data, consumed) = parse(&framed).expect("should parse");
        assert_eq!(id, 0x42);
        assert_eq!(data, &payload);
        assert_eq!(consumed, framed.len());
    }

    #[test]
    fn parse_skips_garbage() {
        let mut buf = vec![0xAA, 0xBB, 0xCC];
        let payload = [0x11u8, 0x22];
        buf.extend_from_slice(&frame(0x01, &payload));
        let (id, data, _) = parse(&buf).expect("should parse");
        assert_eq!(id, 0x01);
        assert_eq!(data, &payload);
    }

    #[test]
    fn empty_payload() {
        let framed = frame(0x04, &[]);
        let (id, data, consumed) = parse(&framed).expect("should parse");
        assert_eq!(id, 0x04);
        assert!(data.is_empty());
        assert_eq!(consumed, framed.len());
    }

    /// Verify OrionCmdPacket encode → wire::frame → wire::parse → decode round-trip.
    #[test]
    fn orion_cmd_packet_roundtrip() {
        let mut pkt = OrionCmdPacket::default();
        pkt.cmd.target = [0.5, -0.3];
        pkt.cmd.mode = OrionMode::OrionModePosition;
        pkt.cmd.stabilized = 1;
        pkt.cmd.impulse_time = 2.5;

        // Encode payload, wrap in Orion frame
        let payload = pkt.encode();
        let framed = frame(OrionCmdPacket::ID, &payload);

        // Parse and decode
        let (id, data, consumed) = parse(&framed).expect("frame should parse");
        assert_eq!(id, OrionCmdPacket::ID);
        assert_eq!(consumed, framed.len());

        let decoded = OrionCmdPacket::decode(data).expect("packet should decode");

        // Targets are scaled to i16 with scaler=1000 → ±0.001 rad precision
        assert!((decoded.cmd.target[0] - pkt.cmd.target[0]).abs() < 0.002,
            "pan: {} vs {}", decoded.cmd.target[0], pkt.cmd.target[0]);
        assert!((decoded.cmd.target[1] - pkt.cmd.target[1]).abs() < 0.002,
            "tilt: {} vs {}", decoded.cmd.target[1], pkt.cmd.target[1]);
        assert_eq!(decoded.cmd.mode, OrionMode::OrionModePosition);
        assert_eq!(decoded.cmd.stabilized, 1);
        // ImpulseTime scaler=10 → ±0.1 s precision
        assert!((decoded.cmd.impulse_time - pkt.cmd.impulse_time).abs() < 0.15,
            "impulse_time: {} vs {}", decoded.cmd.impulse_time, pkt.cmd.impulse_time);
    }
}
