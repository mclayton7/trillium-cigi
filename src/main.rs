mod cigi;
mod config;
mod convert;
mod faults;
mod geo;
mod orion;
mod platform;
mod simulator;
mod trillium;

use std::time::{Duration, Instant};

use config::Config;
use platform::{MavLinkSource, PlatformSource, PlatformState, Stanag4586Source, StaticSource};
use simulator::GimbalSimulator;
use tokio::sync::{mpsc, watch};
use tokio::time::MissedTickBehavior;

use cigi::host::{CigiResponse, build_datagram};
use convert::to_cigi::{make_ig_control, orion_cmd_to_sensor_control, platform_to_entity_control};
use convert::to_orion::sensor_response_to_telemetry;
use orion::GeolocateTelemetryCorePacket;
use orion::OrionCmdPacket;

/// How long without a StartOfFrame before we declare the scene generator gone.
const SG_TIMEOUT: Duration = Duration::from_secs(2);
/// Platform entity ID used in EntityControl packets.
const PLATFORM_ENTITY_ID: u16 = 1;
/// Tick interval (50 Hz).
const DT: f64 = 0.02;

#[tokio::main]
async fn main() {
    let cfg = Config::load("config.toml");

    // ── CLI args ──────────────────────────────────────────────────────────
    let args: Vec<String> = std::env::args().collect();
    let diag_enabled = args.iter().any(|a| a == "--diag");

    // ── Platform state — watch channel seeded from config ────────────────
    let (platform_tx, platform_rx) = watch::channel(PlatformState::from_config(&cfg));
    match cfg.platform_source.as_str() {
        "mavlink"    => tokio::spawn(MavLinkSource::from_config(&cfg).run(platform_tx)),
        "stanag4586" => tokio::spawn(Stanag4586Source::from_config(&cfg).run(platform_tx)),
        _            => tokio::spawn(StaticSource::from_config(&cfg).run(platform_tx)),
    };

    // ── Fallback simulator ────────────────────────────────────────────────
    let mut sim = GimbalSimulator::with_config(cfg.clone());

    // ── Channels ──────────────────────────────────────────────────────────
    let (orion_cmd_tx, mut orion_cmd_rx) = mpsc::channel::<OrionCmdPacket>(32);
    let (orion_telem_tx, orion_telem_rx) = mpsc::channel::<GeolocateTelemetryCorePacket>(4);
    let (cigi_send_tx, cigi_send_rx) = mpsc::channel::<cigi::host::CigiDatagram>(8);
    let (cigi_resp_tx, mut cigi_resp_rx) = mpsc::channel::<CigiResponse>(8);

    // ── Spawn Trillium TCP server task ────────────────────────────────────
    tokio::spawn(trillium::server::run(
        cfg.clone(),
        orion_cmd_tx,
        orion_telem_rx,
    ));

    // ── Spawn CIGI host UDP I/O task ──────────────────────────────────────
    tokio::spawn(cigi::host::run(cfg.clone(), cigi_send_rx, cigi_resp_tx));

    // ── Event loop ────────────────────────────────────────────────────────
    let mut tick = tokio::time::interval(Duration::from_secs_f64(DT));
    tick.set_missed_tick_behavior(MissedTickBehavior::Skip);

    let mut frame_ctr: u32 = 0;
    let mut noise_seed: u32 = 0xCAFE_BABE;
    let mut last_cmd: Option<OrionCmdPacket> = None;
    let mut last_sof: Option<Instant> = None;

    println!(
        "CIGI Trillium Bridge — Trillium TCP :{} | CIGI UDP :{} → {}:{}",
        cfg.orion_listen_port,
        cfg.cigi_listen_port,
        cfg.scene_generator_ip,
        cfg.scene_generator_cigi_port,
    );
    if diag_enabled {
        println!("[diag] diagnostics enabled at 1 Hz");
    }

    loop {
        tokio::select! {
            biased;

            // ── 50 Hz tick ────────────────────────────────────────────────
            _ = tick.tick() => {
                frame_ctr = frame_ctr.wrapping_add(1);
                let sg_connected = last_sof
                    .map(|t| t.elapsed() < SG_TIMEOUT)
                    .unwrap_or(false);

                if sg_connected {
                    // ── Scene generator path ─────────────────────────────
                    let platform = platform_rx.borrow().clone();
                    let ig = make_ig_control(frame_ctr);
                    let ec = platform_to_entity_control(&platform, PLATFORM_ENTITY_ID);
                    let sc = last_cmd.as_ref().map(|cmd| {
                        orion_cmd_to_sensor_control(cmd, sim.camera_index, sim.zoom_level)
                    });
                    let datagram = build_datagram(&ig, &ec, sc.as_ref());
                    cigi_send_tx.try_send(datagram).ok();
                } else {
                    // ── Simulator fallback ───────────────────────────────
                    if let Some(ref cmd) = last_cmd {
                        let sc = orion_cmd_to_sensor_control(cmd, sim.camera_index, sim.zoom_level);
                        sim.apply_sensor_control(&sc);
                    }
                    sim.tick(DT);

                    // Send synthetic telemetry at 10 Hz (every 5 ticks)
                    if frame_ctr % 5 == 0 {
                        let ser = sim.to_sensor_extended_response();
                        let platform = platform_rx.borrow().clone();
                        let telem = sensor_response_to_telemetry(&ser, &platform);
                        orion_telem_tx.try_send(telem).ok();
                    }

                    if diag_enabled && frame_ctr % 50 == 0 {
                        let diag = sim.faults.build_diagnostics(&mut noise_seed);
                        sim.faults.log_diagnostics(&diag);
                    }
                }
            }

            // ── Orion command from Trillium ───────────────────────────────
            Some(cmd) = orion_cmd_rx.recv() => {
                last_cmd = Some(cmd);
            }

            // ── CIGI response from scene generator ───────────────────────
            Some(resp) = cigi_resp_rx.recv() => {
                match resp {
                    CigiResponse::StartOfFrame(_sof) => {
                        last_sof = Some(Instant::now());
                    }
                    CigiResponse::SensorResponse(ser) => {
                        let platform = platform_rx.borrow().clone();
                        let telem = sensor_response_to_telemetry(&ser, &platform);
                        orion_telem_tx.try_send(telem).ok();
                    }
                }
            }
        }
    }
}
