mod cigi;
mod convert;
mod orion;
mod server;
mod simulator;

use server::{CigiPacket, CigiServer};
use simulator::GimbalSimulator;
use std::time::Duration;
use tokio::time::MissedTickBehavior;

#[tokio::main]
async fn main() {
    let mut sim = GimbalSimulator::default();
    let mut srv = CigiServer::new(8008).await.expect("Failed to bind UDP port 8008");

    let mut tick = tokio::time::interval(Duration::from_millis(20)); // 50 Hz
    tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
    let mut frame_ctr: u32 = 0;

    println!("CIGI Trillium Gimbal Simulator — listening on UDP :8008");
    println!("Send CIGI packets to this port to control the simulated gimbal.");

    loop {
        tokio::select! {
            biased;

            _ = tick.tick() => {
                // ── Tick simulator at 50 Hz ──
                sim.tick(0.02);
                frame_ctr = frame_ctr.wrapping_add(1);

                // ── Send Start of Frame every frame (50 Hz) ──
                let sof = sim.to_start_of_frame();
                srv.send(&sof.encode()).await.ok();

                // ── Send Sensor (Extended) Response at 10 Hz (every 5 frames) ──
                if frame_ctr % 5 == 0 {
                    let sr = sim.to_sensor_extended_response();
                    srv.send(&sr.encode()).await.ok();
                }
            }

            pkts = srv.recv_packets() => {
                for pkt in pkts {
                    match pkt {
                        CigiPacket::SensorControl(sc) => sim.apply_sensor_control(&sc),
                        CigiPacket::EntityControl(ec) => sim.apply_entity_control(&ec),
                        CigiPacket::IgControl(igc) => {
                            sim.ig_mode = igc.ig_mode;
                            sim.host_frame_ctr = igc.frame_ctr;
                        }
                        CigiPacket::Unknown(..) => {}
                    }
                }
            }
        }
    }
}
