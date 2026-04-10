mod config;
mod convert;
mod faults;
mod orion;
mod simulator;
mod trillium;

use std::time::{Duration, Instant};

use config::Config;
use simulator::GimbalSimulator;
use tokio::sync::{mpsc, watch};
use tokio::time::MissedTickBehavior;

use sim_core::cigi;
use sim_core::cigi::build::{
    build_view_control, build_wire_sensor_control, make_ig_control, platform_to_entity_control,
};
use sim_core::cigi::host::{CigiResponse, build_datagram};
use convert::to_cigi::orion_cmd_to_sensor_control;
use convert::to_orion::sensor_response_to_telemetry;
use orion::GeolocateTelemetryCorePacket;
use orion::OrionCmdPacket;

/// How long without a StartOfFrame before we declare the scene generator gone.
const SG_TIMEOUT: Duration = Duration::from_secs(2);
/// Platform entity ID used in EntityControl packets.
const PLATFORM_ENTITY_ID: u16 = 1;
/// CIGI view identifier for the sensor view.
const SENSOR_VIEW_ID: u16 = 1;
/// Tick interval (50 Hz).
const DT: f64 = 0.02;

#[tokio::main]
async fn main() {
    let cfg = Config::load("config.toml");

    // ── CLI args ──────────────────────────────────────────────────────────
    let args: Vec<String> = std::env::args().collect();
    let diag_enabled = args.iter().any(|a| a == "--diag");

    // ── Platform state — watch channel seeded from config ────────────────
    let platform_source = cfg.platform_source_config();
    println!("[main] platform source: {}", platform_source.kind());
    let (platform_tx, platform_rx) = watch::channel(platform_source.initial_state());
    platform_source.spawn(platform_tx);

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
    tokio::spawn(cigi::host::run(cfg.cigi_listen_port, cfg.scene_generator_ip.clone(), cfg.scene_generator_cigi_port, cigi_send_rx, cigi_resp_tx));

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
                    //
                    // Drive the Rust-side simulator forward in lock-step with
                    // the scene generator path so `sim.pan`/`sim.tilt` always
                    // reflect the latest Orion command after slew dynamics.
                    // These are what we ship in ViewControl.
                    if let Some(ref cmd) = last_cmd {
                        let sc_internal = orion_cmd_to_sensor_control(cmd);
                        sim.apply_sensor_control(&sc_internal);
                    }
                    sim.tick(DT);

                    let platform = platform_rx.borrow().clone();
                    let ig = make_ig_control(frame_ctr);
                    let ec = platform_to_entity_control(&platform, PLATFORM_ENTITY_ID);
                    // ViewControl: authoritative gimbal pose + mount offsets.
                    // Uses the simulator's current (slewed) pan/tilt so the IG
                    // renders exactly what the Rust side believes.
                    let vc = build_view_control(
                        sim.pan,
                        sim.tilt,
                        &cfg.gimbal_mount,
                        SENSOR_VIEW_ID,
                        PLATFORM_ENTITY_ID,
                    );
                    // SensorControl on the wire: only camera selection + FOV
                    // preset (via Gain). Zoom command plumbing from Orion is
                    // future work — always send preset 0 (widest) for now.
                    // The scene generator reads Gain as preset index; sending
                    // a pan-derived value (old behavior) caused wild FOV jumps.
                    let sc_wire = build_wire_sensor_control(
                        sim.camera_index.max(0) as u8,
                        SENSOR_VIEW_ID as u8,
                        0,
                        3,
                    );
                    let datagram = build_datagram(&ig, &ec, Some(&vc), Some(&sc_wire));
                    cigi_send_tx.try_send(datagram).ok();

                    // Emit synthetic telemetry so the Orion Controller keeps
                    // seeing fresh look-point data while the IG path is up.
                    if frame_ctr % 5 == 0 {
                        let ser = sim.to_sensor_extended_response();
                        let telem = sensor_response_to_telemetry(&ser, &platform);
                        orion_telem_tx.try_send(telem).ok();
                    }
                } else {
                    // ── Simulator fallback ───────────────────────────────
                    if let Some(ref cmd) = last_cmd {
                        let sc = orion_cmd_to_sensor_control(cmd);
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
