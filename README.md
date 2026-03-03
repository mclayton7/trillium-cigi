# cigi_trillium — Trillium Orion Gimbal Simulator

A **CIGI v3.3 network simulator** for the [Trillium Engineering Orion](http://w3.trilliumeng.com/) gimbal family. It lets a CIGI-capable image generator, HMI, or test tool control a *software-simulated* gimbal without physical hardware.

```
┌──────────────────────┐         UDP :8008          ┌──────────────────────┐
│   CIGI Host / HMI    │  ◄──────────────────────►  │  cigi_trillium sim   │
│  (image generator,   │   IgControl, EntityControl  │  ┌────────────────┐  │
│   ground station,    │   SensorControl             │  │ GimbalSimulator│  │
│   test harness)      │                             │  │  (pan/tilt     │  │
│                      │   StartOfFrame (50 Hz)      │  │   slew model)  │  │
│                      │   SensorExtendedResponse    │  └───────┬────────┘  │
│                      │   (10 Hz)                   │         │            │
└──────────────────────┘                             │  ┌──────▼────────┐   │
                                                     │  │ Orion data    │   │
                                                     │  │ model (codegen│   │
                                                     │  │ from XML)     │   │
                                                     │  └───────────────┘   │
                                                     └──────────────────────┘
```

---

## Features

- **Compile-time code generation** — `build.rs` parses `OrionPublicProtocol.xml` (1 700 + lines, 80 + message types) and emits type-safe Rust structs with wire encode/decode for every Orion packet.
- **CIGI v3.3 server** — UDP socket on port 8008 (configurable), non-blocking receive loop, dispatches `IgControl`, `EntityControl`, `SensorControl`.
- **Realistic gimbal dynamics** — pan/tilt slew at ≤ 60 °/s toward commanded position.
- **Periodic telemetry** — `StartOfFrame` at 50 Hz, `SensorExtendedResponse` at 10 Hz.

---

## Building & Running

```bash
# Build (runs code generator then compiles everything)
cargo build --release

# Run (listens on UDP :8008)
./target/release/cigi_trillium
```

Expected output:
```
CIGI Trillium Gimbal Simulator — listening on UDP :8008
Send CIGI packets to this port to control the simulated gimbal.
```

### Quick integration test

```bash
# Send a raw CIGI SensorControl packet (type 17, 24 bytes)
# using netcat — this commands sensor 0 to be active:
python3 - <<'EOF'
import socket, struct
# SensorControl: type=17, size=24, view_id=0, sensor_id=0, state=1 (active), gain=0.75, level=0.5
pkt = bytearray(24)
pkt[0] = 17   # type
pkt[1] = 24   # size
pkt[2] = 0    # view_id
pkt[3] = 0    # sensor_id
pkt[4] = 1    # sensor_state = Active
struct.pack_into('<f', pkt, 6, 0.75)   # gain → pan target
struct.pack_into('<f', pkt, 10, 0.5)   # level → tilt target
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.sendto(bytes(pkt), ('127.0.0.1', 8008))
s.settimeout(0.2)
try:
    data, _ = s.recvfrom(256)
    print(f"Received {len(data)} bytes: type={data[0]}")
except: pass
EOF
```

---

## CIGI Message Reference

### Host → Simulator

| CIGI Message | Type ID | Bytes | Effect |
|---|---|---|---|
| `IgControl` | 1 | 24 | Sets IG mode; simulator tracks host frame counter |
| `EntityControl` | 2 | 48 | Updates platform lat/lon/alt (used in telemetry position) |
| `SensorControl` | 17 | 24 | Commands gimbal pointing and mode (see below) |

#### `SensorControl` → Gimbal State Mapping

| CIGI `sensor_state` | Orion `OrionMode` | Description |
|---|---|---|
| 0 — Inactive | `OrionModeDisabled` | Motors disabled |
| 1 — Active | `OrionModePosition` | Stabilized position control |
| 2 — Tracking | `OrionModeTrack` | Scene/object track mode |

| CIGI field | Mapping |
|---|---|
| `gain` (0–1) | pan target: `(gain*2 - 1) * π` radians |
| `level` (0–1) | tilt target: `(level*2 - 1) * π` radians |

### Simulator → Host

| CIGI Message | Type ID | Bytes | Cadence | Contents |
|---|---|---|---|---|
| `StartOfFrame` | 64 | 16 | 50 Hz (every frame) | IG mode, frame counter, timestamp |
| `SensorExtendedResponse` | 68 | 40 | 10 Hz (every 5th frame) | gate position, entity lat/lon/alt |

#### `GeolocateTelemetryCore` → `SensorExtendedResponse` Mapping

| Orion field | CIGI field | Notes |
|---|---|---|
| `pan` (rad) | `gate_x_pos` | normalized by `hfov` |
| `tilt` (rad) | `gate_y_pos` | normalized by `vfov` |
| `pos_lat` (rad) | `entity_lat` (deg) | converted rad→deg |
| `pos_lon` (rad) | `entity_lon` (deg) | converted rad→deg |
| `pos_alt` (m MSL) | `entity_alt` (m) | direct copy |
| `mode` | `sensor_status` | Active→0, Track→1, Disabled→3 |

---

## Architecture

```
src/
  main.rs             — 50 Hz simulator loop
  cigi/
    messages.rs       — CIGI v3.3 structs (IgControl, EntityControl, SensorControl,
                        StartOfFrame, SensorResponse, SensorExtendedResponse)
  orion/
    mod.rs            — re-exports generated types
    wire.rs           — Orion framing: sync bytes, Fletcher-16 checksum
  convert/
    to_orion.rs       — SensorControl/EntityControl → Orion packets
    to_cigi.rs        — Orion telemetry → CIGI responses
  simulator/
    mod.rs            — GimbalSimulator (pan/tilt slew model)
  server/
    mod.rs            — CigiServer (non-blocking UDP)

build.rs              — XML parser + Rust code generator
OrionPublicProtocol.xml — Trillium Orion protocol schema (source of truth)
```

### Code Generator (`build.rs`)

At `cargo build` time, `build.rs` reads `OrionPublicProtocol.xml` and writes
`$OUT_DIR/orion_generated.rs` containing:

- All **enumerations** with `Default` + `TryFrom<u8>` impls
- All **standalone structures** with `encode(&self, out: &mut Vec<u8>)` / `decode(buf: &mut &[u8])` methods
- All **packets** with `const ID: u8`, `encode(&self) -> Vec<u8>`, `decode(data: &[u8])` methods

Field encoding respects the XML attributes:

| XML attribute | Wire behaviour |
|---|---|
| `scaler="1000"` | `wire = (f64_value * 1000).round() as enc_type` |
| `max="pi"` | `wire = (f64_value / π * MAX_INT).round() as enc_type` |
| `min + max` | linear normalization to `[0, MAX_UINT]` |
| `bitfieldN` | packed into bytes, MSB-first within byte |
| `dependsOn` | `Option<T>` in struct; conditionally encoded |
| `variableArray` | `Vec<T>` in struct; length from preceding field |

---

## Orion Protocol Reference (Internal Data Model)

Key packet categories generated from XML:

| Category | Packet IDs | Description |
|---|---|---|
| **Control** | `0x01` OrionCmd, `0xD5` GeopointCmd, `0xD7` OrionPath | Gimbal pointing commands |
| **Telemetry** | `0xD4` GeolocateTelemetryCore | 10 Hz primary state (pan, tilt, position, mode) |
| **Diagnostics** | `0x41`–`0x46` | Electrical health, performance, vibration, network |
| **Camera** | `0x60`–`0x79` | Camera selection, settings, video options |
| **Navigation** | `0xD0`–`0xD8` | GPS, IMU, INS quality, range |
| **Configuration** | `0x02`, `0x22`, `0xD8`, `0xE4` | UART, limits, INS options, network |

---

## Adding a Real Gimbal (Future Work)

To bridge to a real Orion gimbal over TCP:

1. Add `--gimbal-ip <addr>` CLI flag (e.g., using `std::env::args()`)
2. In `main.rs`, open a `TcpStream` to the gimbal
3. On `SensorControl` receipt: call `convert::to_orion::sensor_control_to_orion_cmd`, encode with `wire::frame`, send over TCP
4. On Orion telemetry receipt: parse with `wire::parse` + `GeolocateTelemetryCorePacket::decode`, convert with `convert::to_cigi`, send to CIGI host
5. Replace `GimbalSimulator` with a thin bridge struct that forwards state from real telemetry

---

## Tests

```bash
cargo test
```

| Test | What it verifies |
|---|---|
| `orion::wire::orion_cmd_packet_roundtrip` | `OrionCmdPacket` encode → `wire::frame` → `wire::parse` → decode (field values preserved within scaler precision) |
| `orion::wire::frame_parse_roundtrip` | Basic framing + checksum |
| `orion::wire::parse_skips_garbage` | Sync-byte scanner |
| `cigi::messages::sensor_control_roundtrip` | CIGI `SensorControl` encode/decode |
| `cigi::messages::start_of_frame_roundtrip` | CIGI `StartOfFrame` encode/decode |
| `simulator::slew_toward_target` | Pan/tilt converge to commanded angle at ≤ 60 °/s |
| `simulator::apply_sensor_control_sets_mode` | Mode and target mapping from `SensorControl` |
| `simulator::sensor_response_end_to_end` | Full pipeline: command → tick → `SensorResponse` |

---

## Viewing Generated Code

```bash
# Inspect the generated Orion types after building:
cat target/debug/build/cigi_trillium-*/out/orion_generated.rs | less
```
