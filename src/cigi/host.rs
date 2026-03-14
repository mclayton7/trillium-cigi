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
fn parse_and_forward(data: &[u8], tx: &mpsc::Sender<CigiResponse>) {
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
