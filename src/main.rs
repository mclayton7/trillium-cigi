mod bridge;
mod cigi;
mod config;
mod convert;
mod faults;
mod geo;
mod orion;
mod server;
mod simulator;

use bridge::GimbalBridge;
use config::Config;
use server::{CigiPacket, CigiServer};
use simulator::GimbalSimulator;
use std::time::Duration;
use tokio::time::MissedTickBehavior;

#[tokio::main]
async fn main() {
    // ── Configuration ─────────────────────────────────────────────────
    let cfg = Config::load("config.toml");

    // ── CLI args ──────────────────────────────────────────────────────
    // Usage: cigi_trillium [--gimbal-ip <host>] [--diag]
    let args: Vec<String> = std::env::args().collect();
    let gimbal_ip = find_arg(&args, "--gimbal-ip");
    let diag_enabled = args.contains(&"--diag".to_string());

    // ── Simulator ─────────────────────────────────────────────────────
    let mut sim = GimbalSimulator::with_config(cfg.clone());

    // ── Network ───────────────────────────────────────────────────────
    let port = cfg.port;
    let mut srv = CigiServer::new(port)
        .await
        .unwrap_or_else(|e| panic!("Failed to bind UDP port {port}: {e}"));

    // ── Optional TCP bridge to real gimbal ────────────────────────────
    let mut bridge: Option<GimbalBridge> = gimbal_ip.map(|ip| GimbalBridge::new(&ip));
    if let Some(ref mut b) = bridge {
        if let Err(e) = b.connect().await {
            eprintln!("[bridge] failed to connect: {e}  (continuing in sim-only mode)");
            bridge = None;
        }
    }

    // ── Event loop ────────────────────────────────────────────────────
    let mut tick = tokio::time::interval(Duration::from_millis(20)); // 50 Hz
    tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
    let mut frame_ctr: u32 = 0;
    let mut noise_seed: u32 = 0xCAFE_BABE;

    println!("CIGI Trillium Gimbal Simulator — UDP :{port}");
    if bridge.is_some() {
        println!("[bridge] TCP bridge active");
    }
    println!("Send CIGI packets to this port to control the simulated gimbal.");
    println!("  Options: --gimbal-ip <host>   proxy to real Orion over TCP");
    println!("           --diag               log diagnostics at 1 Hz");

    loop {
        tokio::select! {
            biased;

            _ = tick.tick() => {
                // ── Tick simulator at 50 Hz ──────────────────────────
                sim.tick(0.02);
                frame_ctr = frame_ctr.wrapping_add(1);

                // ── Send Start of Frame (50 Hz) ──────────────────────
                let sof = sim.to_start_of_frame();
                srv.send(&sof.encode()).await.ok();

                // ── Send Sensor (Extended) Response at 10 Hz ─────────
                if frame_ctr % 5 == 0 {
                    let sr = sim.to_sensor_extended_response();
                    srv.send(&sr.encode()).await.ok();
                }

                // ── Diagnostics at 1 Hz ──────────────────────────────
                if diag_enabled && frame_ctr % 50 == 0 {
                    let diag = sim.faults.build_diagnostics(&mut noise_seed);
                    sim.faults.log_diagnostics(&diag);
                }

                // ── TCP bridge: reconnect if needed, apply real telemetry ──
                if let Some(ref mut b) = bridge {
                    b.try_reconnect().await;
                    if let Some(telem) = b.recv_telemetry().await {
                        sim.apply_hardware_telemetry(&telem);
                    }
                }
            }

            pkts = srv.recv_packets() => {
                for pkt in pkts {
                    match pkt {
                        CigiPacket::SensorControl(ref sc) => {
                            sim.apply_sensor_control(sc);
                            // Forward to real gimbal if bridge is active.
                            if let Some(ref mut b) = bridge {
                                let cmd = convert::to_orion::sensor_control_to_orion_cmd(sc);
                                b.send_cmd(&cmd).await.ok();
                            }
                        }
                        CigiPacket::EntityControl(ref ec) => {
                            sim.apply_entity_control(ec);
                        }
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

/// Find the value after a named CLI argument, e.g. `--gimbal-ip 192.168.1.10`.
fn find_arg(args: &[String], flag: &str) -> Option<String> {
    args.windows(2)
        .find(|w| w[0] == flag)
        .map(|w| w[1].clone())
}
