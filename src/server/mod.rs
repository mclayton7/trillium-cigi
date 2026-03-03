// CIGI UDP server — binds to 0.0.0.0:PORT, receives CIGI host packets,
// sends IG response packets back to the learned host address.

use std::net::SocketAddr;
use tokio::net::UdpSocket;

use crate::cigi::messages::{EntityControl, IgControl, SensorControl};

/// Parsed CIGI packets we care about.
#[derive(Debug)]
pub enum CigiPacket {
    IgControl(IgControl),
    EntityControl(EntityControl),
    SensorControl(SensorControl),
    Unknown(u8, Vec<u8>),
}

/// CIGI v3.3 UDP network server.
pub struct CigiServer {
    socket: UdpSocket,
    host_addr: Option<SocketAddr>,
}

impl CigiServer {
    /// Bind to `0.0.0.0:port`.
    pub async fn new(port: u16) -> std::io::Result<Self> {
        let socket = UdpSocket::bind(format!("0.0.0.0:{port}")).await?;
        Ok(Self { socket, host_addr: None })
    }

    /// Await one UDP datagram and return all parseable CIGI packets it contains.
    pub async fn recv_packets(&mut self) -> Vec<CigiPacket> {
        let mut buf = [0u8; 65536];
        match self.socket.recv_from(&mut buf).await {
            Ok((n, addr)) => {
                self.host_addr = Some(addr);
                parse_cigi_datagram(&buf[..n])
            }
            Err(_) => vec![],
        }
    }

    /// Send raw bytes to the current host address (if one is known).
    pub async fn send(&self, data: &[u8]) -> std::io::Result<()> {
        if let Some(addr) = self.host_addr {
            self.socket.send_to(data, addr).await?;
        }
        Ok(())
    }

    /// The remote address that most recently sent us a packet.
    pub fn host_addr(&self) -> Option<SocketAddr> {
        self.host_addr
    }
}

/// Parse a single CIGI datagram (may contain multiple packets).
fn parse_cigi_datagram(data: &[u8]) -> Vec<CigiPacket> {
    let mut packets = Vec::new();
    let mut offset = 0;

    while offset + 2 <= data.len() {
        let type_id = data[offset];
        let size = data[offset + 1] as usize;

        if size < 2 || offset + size > data.len() {
            break;
        }

        let pkt_data = &data[offset..offset + size];

        let pkt = match type_id {
            IgControl::TYPE_ID => IgControl::decode(pkt_data).map(CigiPacket::IgControl),
            EntityControl::TYPE_ID => EntityControl::decode(pkt_data).map(CigiPacket::EntityControl),
            SensorControl::TYPE_ID => SensorControl::decode(pkt_data).map(CigiPacket::SensorControl),
            _ => Some(CigiPacket::Unknown(type_id, pkt_data.to_vec())),
        };

        if let Some(p) = pkt {
            packets.push(p);
        }

        offset += size;
    }

    packets
}
