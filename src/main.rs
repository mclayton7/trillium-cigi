mod config;
mod convert;
mod faults;
mod orion;
mod simulator;
mod trillium;

use std::time::{Duration, Instant};

use config::{Config, RuntimeMode};
use simulator::GimbalSimulator;
use tokio::sync::{mpsc, watch};
use tokio::time::MissedTickBehavior;

use sim_core::cigi;
use sim_core::cigi::build::{
    build_view_control, build_wire_sensor_control, make_ig_control, platform_to_entity_control,
};
use sim_core::cigi::host::{CigiResponse, build_datagram};
use sim_core::cigi::messages::IgControl;
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
/// Number of discrete CIGI FOV presets advertised on the wire (matches the IG).
const WIRE_PRESET_COUNT: u8 = 3;
/// How fresh a SensorResponse must be before we suppress the synthetic-telemetry keepalive.
const SR_KEEPALIVE_STALE: Duration = Duration::from_millis(200);

#[tokio::main]
async fn main() {
    let cfg = Config::load("config.toml");

    // ── CLI args ──────────────────────────────────────────────────────────
    let args: Vec<String> = std::env::args().collect();
    let diag_enabled = args.iter().any(|a| a == "--diag");
    // --simulator / --bridge override [runtime] mode in the config file.
    // Last-wins on duplicates.
    let cli_mode_override = args.iter().rev().find_map(|a| match a.as_str() {
        "--simulator" => Some(RuntimeMode::Simulator),
        "--bridge" => Some(RuntimeMode::Bridge),
        _ => None,
    });
    let cfg = match cli_mode_override {
        Some(m) => cfg.with_runtime_mode(m),
        None => cfg,
    };

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

    // ── Bind Trillium TCP listener up front so port-in-use fails fast ─────
    // Previously the server task was spawned and bound asynchronously, so a
    // bind failure crashed only the spawned task — main kept ticking with no
    // Trillium connectivity. Bind here and exit cleanly on failure.
    let trillium_listener = match trillium::server::bind(&cfg).await {
        Ok(l) => l,
        Err(e) => {
            eprintln!(
                "[main] failed to bind Trillium listener on port {}: {e}",
                cfg.orion_listen_port
            );
            std::process::exit(1);
        }
    };
    tokio::spawn(trillium::server::serve(
        trillium_listener,
        orion_cmd_tx,
        orion_telem_rx,
    ));

    // ── CIGI host UDP I/O — bridge mode only ──────────────────────────────
    // In simulator mode we skip the bind and the spawn entirely so the UDP
    // port is free (multiple sims can run on one host) and there is no SG
    // detection overhead.
    let (cigi_send_tx, mut cigi_resp_rx): (
        Option<mpsc::Sender<cigi::host::CigiDatagram>>,
        Option<mpsc::Receiver<CigiResponse>>,
    ) = match cfg.runtime_mode {
        RuntimeMode::Bridge => {
            let (send_tx, send_rx) = mpsc::channel::<cigi::host::CigiDatagram>(8);
            let (resp_tx, resp_rx) = mpsc::channel::<CigiResponse>(8);
            tokio::spawn(cigi::host::run(
                cfg.cigi_listen_port,
                cfg.scene_generator_ip.clone(),
                cfg.scene_generator_cigi_port,
                send_rx,
                resp_tx,
            ));
            (Some(send_tx), Some(resp_rx))
        }
        RuntimeMode::Simulator => (None, None),
    };

    // ── Event loop ────────────────────────────────────────────────────────
    let mut tick = tokio::time::interval(Duration::from_secs_f64(DT));
    tick.set_missed_tick_behavior(MissedTickBehavior::Skip);

    let startup_time = Instant::now();
    let mut frame_ctr: u32 = 0;
    let mut noise_seed: u32 = 0xCAFE_BABE;
    let mut last_cmd: Option<OrionCmdPacket> = None;
    let mut last_sof: Option<Instant> = None;
    let mut last_ig_frame_ctr: u32 = 0;
    // Most recent SensorResponse receipt. Used to suppress the synthetic
    // keepalive when the scene generator is already driving telemetry, so the
    // Orion client doesn't see interleaved lookpoints from two sources.
    let mut last_sensor_response: Option<Instant> = None;

    match cfg.runtime_mode {
        RuntimeMode::Bridge => println!(
            "CIGI Trillium Bridge — mode = bridge — Trillium TCP :{} | CIGI UDP :{} → {}:{}",
            cfg.orion_listen_port,
            cfg.cigi_listen_port,
            cfg.scene_generator_ip,
            cfg.scene_generator_cigi_port,
        ),
        RuntimeMode::Simulator => println!(
            "CIGI Trillium Bridge — mode = simulator — Trillium TCP :{} (CIGI disabled)",
            cfg.orion_listen_port,
        ),
    }
    if diag_enabled {
        println!("[diag] diagnostics enabled at 1 Hz");
    }

    loop {
        tokio::select! {
            biased;

            // ── 50 Hz tick ────────────────────────────────────────────────
            _ = tick.tick() => {
                frame_ctr = frame_ctr.wrapping_add(1);
                // In simulator mode, cigi_resp_rx is None and last_sof can
                // never advance — `sg_connected` is always false, which
                // forces the simulator-fallback arm to run every tick.
                let sg_connected = matches!(cfg.runtime_mode, RuntimeMode::Bridge)
                    && last_sof
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
                        let sc_internal =
                            orion_cmd_to_sensor_control(cmd, sim.camera_index, sim.zoom_level);
                        sim.apply_sensor_control(&sc_internal);
                    }
                    sim.tick(DT);

                    let platform = platform_rx.borrow().clone();
                    // Build IgControl with the IG's most recent frame counter
                    // (echoed back for drop detection) and host elapsed time.
                    let ig = IgControl {
                        last_rcvd_ig_frame_ctr: last_ig_frame_ctr as u16,
                        timestamp_valid: true,
                        timestamp: startup_time.elapsed().as_secs_f64(),
                        ..make_ig_control(frame_ctr)
                    };
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
                    // SensorControl on the wire: camera selection + FOV preset
                    // (via Gain). The scene generator reads Gain as a preset
                    // index; zoom_level ∈ [0,1] maps uniformly onto the
                    // WIRE_PRESET_COUNT-many presets.
                    let preset_index = ((sim.zoom_level.clamp(0.0, 1.0)
                        * WIRE_PRESET_COUNT as f32) as u8)
                        .min(WIRE_PRESET_COUNT - 1);
                    let sc_wire = build_wire_sensor_control(
                        sim.camera_index.max(0) as u8,
                        SENSOR_VIEW_ID as u8,
                        preset_index,
                        WIRE_PRESET_COUNT,
                    );
                    let datagram = build_datagram(&ig, &ec, Some(&vc), Some(&sc_wire));
                    if let Some(tx) = cigi_send_tx.as_ref() {
                        tx.try_send(datagram).ok();
                    }

                    // Synthetic telemetry acts as a keepalive only when the
                    // scene generator isn't currently producing SensorResponse
                    // frames. If a real response arrived within
                    // SR_KEEPALIVE_STALE, skip — the SR arm below is already
                    // forwarding authoritative telemetry.
                    let sr_fresh = last_sensor_response
                        .map(|t| t.elapsed() < SR_KEEPALIVE_STALE)
                        .unwrap_or(false);
                    if frame_ctr % 5 == 0 && !sr_fresh {
                        let ser = sim.to_sensor_extended_response();
                        let telem = sensor_response_to_telemetry(&ser, &platform);
                        orion_telem_tx.try_send(telem).ok();
                    }
                } else {
                    // ── Simulator fallback (and pure-simulator mode) ─────
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

            // ── CIGI response from scene generator (bridge mode only) ────
            // In simulator mode `cigi_resp_rx` is None; the inner future
            // resolves to `pending()` and never fires.
            Some(resp) = async {
                match cigi_resp_rx.as_mut() {
                    Some(rx) => rx.recv().await,
                    None => std::future::pending().await,
                }
            } => {
                match resp {
                    CigiResponse::StartOfFrame(sof) => {
                        last_sof = Some(Instant::now());
                        last_ig_frame_ctr = sof.ig_frame_ctr;
                    }
                    CigiResponse::SensorResponse(ser) => {
                        last_sensor_response = Some(Instant::now());
                        let platform = platform_rx.borrow().clone();
                        let telem = sensor_response_to_telemetry(&ser, &platform);
                        orion_telem_tx.try_send(telem).ok();
                    }
                }
            }
        }
    }
}
