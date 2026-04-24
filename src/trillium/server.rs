// Trillium TCP server — accepts Orion protocol connections from a Trillium controller.
//
// Listens on `orion_listen_port` (default 8008).  Accepts one connection at a
// time.  For each connection:
//   - reads Orion frames (big-endian, orion::wire framing) → OrionCmdPacket
//     → sends to `orion_cmd_tx`
//   - receives GeolocateTelemetryCorePacket from `orion_telem_rx`
//     → encodes and writes back over TCP
// On disconnect, waits for the next incoming connection.

use std::io;

use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::mpsc;

use crate::config::Config;
use crate::orion::wire;
use crate::orion::{GeolocateTelemetryCorePacket, OrionCmdPacket};

/// Maximum bytes buffered from a single TCP connection before disconnecting.
/// Guards against a misbehaving client filling memory with unparseable data.
const MAX_RX_BUF: usize = 65_536;

/// Bind the Trillium listener up front so a port-in-use failure is reported
/// to the caller before any background task is spawned.
pub async fn bind(cfg: &Config) -> io::Result<TcpListener> {
    let addr = format!("0.0.0.0:{}", cfg.orion_listen_port);
    let listener = TcpListener::bind(&addr).await?;
    println!("[trillium] TCP server listening on {addr}");
    Ok(listener)
}

/// Service connections on an already-bound listener forever.
///
/// - `orion_cmd_tx`: decoded `OrionCmdPacket`s are sent here.
/// - `orion_telem_rx`: `GeolocateTelemetryCorePacket`s received here are forwarded to the client.
pub async fn serve(
    listener: TcpListener,
    orion_cmd_tx: mpsc::Sender<OrionCmdPacket>,
    mut orion_telem_rx: mpsc::Receiver<GeolocateTelemetryCorePacket>,
) {
    loop {
        match listener.accept().await {
            Ok((stream, peer)) => {
                println!("[trillium] connection from {peer}");
                handle_connection(stream, &orion_cmd_tx, &mut orion_telem_rx).await;
                println!("[trillium] {peer} disconnected — waiting for next connection");
            }
            Err(e) => eprintln!("[trillium] accept error: {e}"),
        }
    }
}

/// Service a single TCP connection until it closes.
async fn handle_connection(
    stream: TcpStream,
    orion_cmd_tx: &mpsc::Sender<OrionCmdPacket>,
    orion_telem_rx: &mut mpsc::Receiver<GeolocateTelemetryCorePacket>,
) {
    stream.set_nodelay(true).ok();
    let (mut reader, mut writer) = stream.into_split();

    let mut rx_buf: Vec<u8> = Vec::with_capacity(4096);
    let mut read_chunk = [0u8; 4096];

    loop {
        tokio::select! {
            // ── Telemetry → TCP ─────────────────────────────────────────────
            telem = orion_telem_rx.recv() => {
                let Some(pkt) = telem else { return };
                let payload = pkt.encode();
                let framed = wire::frame(GeolocateTelemetryCorePacket::ID, &payload);
                if writer.write_all(&framed).await.is_err() {
                    return;
                }
            }

            // ── TCP → commands ──────────────────────────────────────────────
            result = reader.read(&mut read_chunk) => {
                match result {
                    Ok(0) | Err(_) => return,
                    Ok(n) => {
                        rx_buf.extend_from_slice(&read_chunk[..n]);
                        if rx_buf.len() > MAX_RX_BUF {
                            eprintln!("[trillium] rx_buf overflow — disconnecting");
                            return;
                        }
                        let mut consumed = 0;
                        while let Some((id, data, bytes)) = wire::parse(&rx_buf[consumed..]) {
                            consumed += bytes;
                            if id == OrionCmdPacket::ID {
                                if let Some(cmd) = OrionCmdPacket::decode(data) {
                                    orion_cmd_tx.send(cmd).await.ok();
                                }
                            }
                        }
                        if consumed > 0 {
                            rx_buf.drain(..consumed);
                        }
                    }
                }
            }
        }
    }
}
