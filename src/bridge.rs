// Orion TCP bridge — Phase 7.2.
//
// Optional `--gimbal-ip <host>` mode: forwards Orion commands over TCP to a real
// Trillium Orion gimbal and relays its telemetry back to the CIGI host.
//
// The Orion hardware listens on TCP port 8008 using the same wire format as the
// generated `orion::wire` framing module.

use std::time::{Duration, Instant};

use tokio::io::AsyncWriteExt;
use tokio::net::TcpStream;

use crate::orion::wire;
use crate::orion::{GeolocateTelemetryCorePacket, OrionCmdPacket};

// Orion hardware always uses TCP port 8008.
const ORION_TCP_PORT: u16 = 8008;
/// How long to wait between reconnection attempts.
const RECONNECT_INTERVAL: Duration = Duration::from_secs(5);

/// TCP connection to a real Orion gimbal.
pub struct GimbalBridge {
    addr: String,
    stream: Option<TcpStream>,
    rx_buf: Vec<u8>,
    last_connect_attempt: Option<Instant>,
}

impl GimbalBridge {
    /// Create a bridge pointed at `host` (IP or hostname; port is always 8008).
    pub fn new(host: &str) -> Self {
        Self {
            addr: format!("{host}:{ORION_TCP_PORT}"),
            stream: None,
            rx_buf: Vec::with_capacity(4096),
            last_connect_attempt: None,
        }
    }

    /// Establish the TCP connection.
    pub async fn connect(&mut self) -> std::io::Result<()> {
        self.last_connect_attempt = Some(Instant::now());
        let stream = TcpStream::connect(&self.addr).await?;
        stream.set_nodelay(true)?;
        self.stream = Some(stream);
        println!("[bridge] connected to {}", self.addr);
        Ok(())
    }

    /// If disconnected and the reconnect interval has elapsed, attempt to
    /// reconnect.  Silently does nothing if already connected or too soon.
    pub async fn try_reconnect(&mut self) {
        if self.stream.is_some() {
            return;
        }
        let ready = self
            .last_connect_attempt
            .map(|t| t.elapsed() >= RECONNECT_INTERVAL)
            .unwrap_or(true);
        if ready {
            if let Err(e) = self.connect().await {
                eprintln!("[bridge] reconnect to {} failed: {e}", self.addr);
            }
        }
    }

    /// Send an `OrionCmdPacket` to the gimbal over TCP.
    pub async fn send_cmd(&mut self, cmd: &OrionCmdPacket) -> std::io::Result<()> {
        let Some(ref mut stream) = self.stream else {
            return Ok(());
        };
        let payload = cmd.encode();
        let framed = wire::frame(OrionCmdPacket::ID, &payload);
        if let Err(e) = stream.write_all(&framed).await {
            eprintln!("[bridge] write error: {e}");
            self.stream = None;
            return Err(e);
        }
        Ok(())
    }

    /// Non-blocking read: drain any available bytes from the TCP stream and
    /// return the most recently complete `GeolocateTelemetryCorePacket`, if any.
    pub async fn recv_telemetry(&mut self) -> Option<GeolocateTelemetryCorePacket> {
        let stream = self.stream.as_mut()?;

        // Try to read available bytes without blocking the event loop.
        let mut chunk = [0u8; 2048];
        match stream.try_read(&mut chunk) {
            Ok(0) => {
                // EOF — gimbal disconnected
                eprintln!("[bridge] connection closed by remote");
                self.stream = None;
                return None;
            }
            Ok(n) => {
                self.rx_buf.extend_from_slice(&chunk[..n]);
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                // Nothing available right now
            }
            Err(e) => {
                eprintln!("[bridge] read error: {e}");
                self.stream = None;
                return None;
            }
        }

        // Parse all complete frames; keep only the last telemetry packet.
        let mut last_telem: Option<GeolocateTelemetryCorePacket> = None;
        let mut consumed_total = 0;

        while let Some((id, data, consumed)) = wire::parse(&self.rx_buf[consumed_total..]) {
            consumed_total += consumed;
            if id == GeolocateTelemetryCorePacket::ID {
                last_telem = GeolocateTelemetryCorePacket::decode(data);
            }
        }

        if consumed_total > 0 {
            self.rx_buf.drain(..consumed_total);
        }

        last_telem
    }
}
