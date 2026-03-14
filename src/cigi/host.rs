// CIGI host UDP I/O — sends CIGI packets to the scene generator and receives
// StartOfFrame / SensorExtendedResponse responses from it.
//
// Bound to `cigi_listen_port` (default 8101).
// Sends to `scene_generator_ip:scene_generator_cigi_port` (default 127.0.0.1:8100).

use tokio::net::UdpSocket;
use tokio::sync::mpsc;

use crate::cigi::messages::{
    EntityControl, IgControl, SensorControl, SensorExtendedResponse, StartOfFrame,
};
use crate::config::Config;

/// A batch of encoded CIGI packets (IgControl + EntityControl + optional SensorControl)
/// ready for transmission as a single UDP datagram.
pub type CigiDatagram = Vec<u8>;

/// Parsed responses received from the scene generator.
#[derive(Debug)]
pub enum CigiResponse {
    StartOfFrame(StartOfFrame),
    SensorResponse(SensorExtendedResponse),
}

/// Run the CIGI host I/O loop forever.
///
/// - `cigi_send_rx`: encoded CIGI datagrams to send to the scene generator.
/// - `cigi_resp_tx`: parsed responses received from the scene generator.
pub async fn run(
    cfg: Config,
    mut cigi_send_rx: mpsc::Receiver<CigiDatagram>,
    cigi_resp_tx: mpsc::Sender<CigiResponse>,
) {
    let bind_addr = format!("0.0.0.0:{}", cfg.cigi_listen_port);
    let socket = UdpSocket::bind(&bind_addr)
        .await
        .unwrap_or_else(|e| panic!("[cigi_host] failed to bind {bind_addr}: {e}"));
    let dest = format!("{}:{}", cfg.scene_generator_ip, cfg.scene_generator_cigi_port);

    println!("[cigi_host] UDP on {bind_addr}, sending to {dest}");

    let mut recv_buf = [0u8; 65536];

    loop {
        tokio::select! {
            biased;
            // ── Send to scene generator ──────────────────────────────────
            datagram = cigi_send_rx.recv() => {
                let Some(bytes) = datagram else { return };
                socket.send_to(&bytes, &dest).await.ok();
            }

            // ── Receive from scene generator ─────────────────────────────
            result = socket.recv_from(&mut recv_buf) => {
                if let Ok((n, _)) = result {
                    parse_and_forward(&recv_buf[..n], &cigi_resp_tx);
                }
            }
        }
    }
}

/// Parse a CIGI datagram and forward recognised packets to `tx`.
///
/// Uses `try_send` so the UDP I/O task is never stalled waiting for the main loop.
/// Packets are silently dropped if the response channel is full.
pub(crate) fn parse_and_forward(data: &[u8], tx: &mpsc::Sender<CigiResponse>) {
    let mut offset = 0;
    while offset + 2 <= data.len() {
        let type_id = data[offset];
        let size = data[offset + 1] as usize;
        if size < 2 || offset + size > data.len() {
            break;
        }
        let pkt_data = &data[offset..offset + size];
        match type_id {
            StartOfFrame::TYPE_ID => {
                if let Some(sof) = StartOfFrame::decode(pkt_data) {
                    tx.try_send(CigiResponse::StartOfFrame(sof)).ok();
                }
            }
            SensorExtendedResponse::TYPE_ID => {
                if let Some(ser) = SensorExtendedResponse::decode(pkt_data) {
                    tx.try_send(CigiResponse::SensorResponse(ser)).ok();
                }
            }
            _ => {}
        }
        offset += size;
    }
}

// ── Helpers used by main.rs to build outgoing CIGI datagrams ──────────────

/// Encode an `IgControl` + `EntityControl` + optional `SensorControl` into a
/// single CIGI datagram ready for UDP transmission.
pub fn build_datagram(
    ig: &IgControl,
    ec: &EntityControl,
    sc: Option<&SensorControl>,
) -> CigiDatagram {
    let mut out = ig.encode();
    out.extend_from_slice(&ec.encode());
    if let Some(s) = sc {
        out.extend_from_slice(&s.encode());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cigi::messages::{EntityControl, IgControl, SensorControl, SensorExtendedResponse, StartOfFrame};

    // ── build_datagram ────────────────────────────────────────────────────

    #[test]
    fn build_datagram_without_sc() {
        let dg = build_datagram(&IgControl::default(), &EntityControl::default(), None);
        assert_eq!(dg.len(), 72); // 24 (IgControl) + 48 (EntityControl)
        assert_eq!(dg[0], IgControl::TYPE_ID);
        assert_eq!(dg[24], EntityControl::TYPE_ID);
    }

    #[test]
    fn build_datagram_with_sc() {
        let dg = build_datagram(
            &IgControl::default(),
            &EntityControl::default(),
            Some(&SensorControl::default()),
        );
        assert_eq!(dg.len(), 96); // 72 + 24
        assert_eq!(dg[72], SensorControl::TYPE_ID);
    }

    // ── parse_and_forward ─────────────────────────────────────────────────

    fn make_channel() -> (mpsc::Sender<CigiResponse>, mpsc::Receiver<CigiResponse>) {
        mpsc::channel(16)
    }

    #[test]
    fn parse_empty_datagram_produces_nothing() {
        let (tx, mut rx) = make_channel();
        parse_and_forward(&[], &tx);
        assert!(rx.try_recv().is_err());
    }

    #[test]
    fn parse_start_of_frame() {
        let sof = StartOfFrame { ig_frame_ctr: 77, ..Default::default() };
        let (tx, mut rx) = make_channel();
        parse_and_forward(&sof.encode(), &tx);
        match rx.try_recv().unwrap() {
            CigiResponse::StartOfFrame(s) => assert_eq!(s.ig_frame_ctr, 77),
            _ => panic!("expected StartOfFrame"),
        }
    }

    #[test]
    fn parse_sensor_extended_response() {
        // Build a valid 48-byte SensorExtendedResponse buffer
        let mut buf = vec![0u8; 48];
        buf[0] = SensorExtendedResponse::TYPE_ID;
        buf[1] = 48;
        buf[2..4].copy_from_slice(&5u16.to_le_bytes()); // view_id = 5
        buf[4] = 2; // sensor_id = 2
        let (tx, mut rx) = make_channel();
        parse_and_forward(&buf, &tx);
        match rx.try_recv().unwrap() {
            CigiResponse::SensorResponse(r) => assert_eq!(r.sensor_id, 2),
            _ => panic!("expected SensorResponse"),
        }
    }

    #[test]
    fn parse_concatenated_packets() {
        // SOF (24 bytes) followed by SensorExtendedResponse (48 bytes)
        let sof = StartOfFrame { ig_frame_ctr: 1, ..Default::default() };
        let mut ser_buf = vec![0u8; 48];
        ser_buf[0] = SensorExtendedResponse::TYPE_ID;
        ser_buf[1] = 48;
        let mut data = sof.encode();
        data.extend_from_slice(&ser_buf);

        let (tx, mut rx) = make_channel();
        parse_and_forward(&data, &tx);

        assert!(matches!(rx.try_recv().unwrap(), CigiResponse::StartOfFrame(_)));
        assert!(matches!(rx.try_recv().unwrap(), CigiResponse::SensorResponse(_)));
    }

    #[test]
    fn parse_unknown_type_skipped() {
        // Unknown type_id=99, size=8, followed by a valid SOF
        let sof = StartOfFrame { ig_frame_ctr: 42, ..Default::default() };
        let mut data = vec![99u8, 8, 0, 0, 0, 0, 0, 0];
        data.extend_from_slice(&sof.encode());

        let (tx, mut rx) = make_channel();
        parse_and_forward(&data, &tx);

        match rx.try_recv().unwrap() {
            CigiResponse::StartOfFrame(s) => assert_eq!(s.ig_frame_ctr, 42),
            _ => panic!("expected StartOfFrame after skipping unknown"),
        }
    }

    #[test]
    fn parse_truncated_stops_cleanly() {
        // Valid SOF followed by a truncated SER (claims 48 but only 10 bytes available)
        let sof = StartOfFrame::default();
        let mut data = sof.encode();
        data.extend_from_slice(&[SensorExtendedResponse::TYPE_ID, 48, 0, 0, 0, 0, 0, 0, 0, 0]);

        let (tx, mut rx) = make_channel();
        parse_and_forward(&data, &tx); // must not panic
        // SOF should be forwarded; truncated SER silently dropped
        assert!(matches!(rx.try_recv().unwrap(), CigiResponse::StartOfFrame(_)));
        assert!(rx.try_recv().is_err());
    }
}
