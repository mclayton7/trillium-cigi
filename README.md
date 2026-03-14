# cigi_trillium — Trillium Orion Gimbal Simulator

A **CIGI v3.3 UDP simulator** for the Trillium Engineering Orion EO/IR gimbal family. It lets any CIGI-capable image generator, ground station, or test tool drive a fully-modelled virtual gimbal — complete with hardware kinematics, WGS84 geolocation, multi-camera simulation, and fault injection — without physical hardware.

Optionally, the same process can act as a **transparent TCP proxy** to a real Orion gimbal, translating CIGI ↔ Orion wire protocol in real time.

```
┌──────────────────────┐   UDP :8008 (CIGI v3.3)   ┌─────────────────────────────┐
│   CIGI Host / HMI    │ ◄────────────────────────► │      cigi_trillium          │
│                      │   IgControl                 │                             │
│  image generator,    │   EntityControl             │  ┌───────────────────────┐ │
│  ground station,     │   SensorControl             │  │   GimbalSimulator     │ │
│  test harness        │                             │  │                       │ │
│                      │   StartOfFrame  (50 Hz)     │  │  accel/decel profile  │ │
│                      │   SensorExtendedResponse    │  │  WGS84 geolocation    │ │
│                      │   (10 Hz)                   │  │  vibration / jitter   │ │
└──────────────────────┘                             │  │  fault injection      │ │
                                                     │  └──────────┬────────────┘ │
                                                     │             │ optional      │
                                                     │  ┌──────────▼────────────┐ │
                                                     │  │   GimbalBridge        │ │
                                                     │  │  TCP :8008 (Orion)    │ │
                                                     │  └──────────┬────────────┘ │
                                                     └─────────────┼───────────────┘
                                                                   │ TCP (Orion wire)
                                                     ┌─────────────▼───────────────┐
                                                     │   Real Orion gimbal          │
                                                     └──────────────────────────────┘
```

---

## Feature Summary

| Feature                  | Detail                                                                                   |
| ------------------------ | ---------------------------------------------------------------------------------------- |
| CIGI v3.3 UDP server     | Port configurable in `config.toml`; learns host address from first packet                |
| Trapezoidal slew profile | Configurable max rate + acceleration; no instantaneous jumps                             |
| Mechanical angle limits  | ±170° pan, −110°/+30° tilt by default; fully configurable; 360° continuous-pan supported |
| Rate mode                | `OrionModeRate` — gain/level command continuous angular rates                            |
| Geopoint mode            | `OrionModeGeopoint` — aims gimbal at a commanded WGS84 coordinate                        |
| Track mode               | `OrionModeTrack` — proportional controller; simulates track loss at FOV edge             |
| WGS84 geolocation        | Platform position + gimbal angles → ground look-point; populates `entity_lat/lon/alt`    |
| Platform stabilisation   | EntityControl roll/pitch/yaw → `gimbal_quat` + `ins_quat` in telemetry                   |
| Vibration / jitter       | Sinusoidal airframe vibration + LCG white-noise floor; amplitude scales with slew rate   |
| Zoom / FOV simulation    | Zoom level 0–1 maps to configurable wide/narrow FOV pair                                 |
| Multi-camera             | 3 cameras (EO wide, EO narrow, IR) selectable by `sensor_id`; 200 ms switch blackout     |
| Fault injection          | GPS loss, motor fault, IMU dropout, thermal warning — API on `sim.faults`                |
| Diagnostics              | `--diag` flag logs voltages, currents, temps at 1 Hz                                     |
| Runtime configuration    | `config.toml` — no recompile needed to tune any parameter                                |
| TCP bridge               | `--gimbal-ip <host>` — proxies commands to real hardware, applies real telemetry         |
| Auto-reconnect           | Bridge retries every 5 s if the TCP connection drops                                     |
| Compile-time codegen     | `build.rs` generates typed Rust structs for all 80+ Orion protocol packets from XML      |

---

## Quick Start

```bash
# Build (runs code generator, then compiles)
cargo build --release

# Run with defaults (UDP :8008, pure simulation)
./target/release/cigi_trillium

# Run with custom config
cp config.toml my_config.toml   # edit as needed
# (config.toml in the working directory is loaded automatically)

# Run against a real Orion gimbal over TCP
./target/release/cigi_trillium --gimbal-ip 192.168.1.42

# Enable 1 Hz diagnostic logging
./target/release/cigi_trillium --diag

# Combine flags
./target/release/cigi_trillium --gimbal-ip 192.168.1.42 --diag
```

Expected startup output:
```
CIGI Trillium Gimbal Simulator — UDP :8008
Send CIGI packets to this port to control the simulated gimbal.
  Options: --gimbal-ip <host>   proxy to real Orion over TCP
           --diag               log diagnostics at 1 Hz
```

---

## Configuration (`config.toml`)

`config.toml` is loaded from the working directory at startup. Missing file → built-in defaults. Unknown keys are silently ignored. Section headers (`[network]` etc.) are cosmetic — only key = value lines matter.

```toml
[network]
port = 8008                  # UDP listen port

[kinematics]
max_slew_rate_deg_s = 60.0   # peak angular velocity (°/s)
max_accel_deg_s2    = 300.0  # acceleration limit (°/s²)
pan_limit_deg       = 170.0  # symmetric pan hard-stop (set to 360 for continuous)
tilt_min_deg        = -110.0 # tilt lower limit
tilt_max_deg        = 30.0   # tilt upper limit

[camera]
hfov_wide_deg   = 30.0       # wide-end (zoom = 0) horizontal FOV
vfov_wide_deg   = 22.5
hfov_narrow_deg = 3.0        # narrow-end (zoom = 1) horizontal FOV
vfov_narrow_deg = 2.25

[vibration]
jitter_freq_hz       = 10.0  # sinusoidal vibration frequency
jitter_amplitude_deg = 0.05  # peak sinusoidal amplitude
noise_floor_deg      = 0.01  # white-noise RMS floor
```

A pre-built profile for the **Trillium HD45-LV-CZ-GS** (360° pan, −80°/+42° tilt, 46.8°→1.2° EO zoom) is provided in `config_hd45.toml`. Rename or copy it to `config.toml` to use it.

### Continuous-pan mode

Set `pan_limit_deg = 360` (or greater) to enable continuous rotation. The simulator will:
- Never clamp or flag pan-limit exceeded
- Compute the shortest angular path when a new position is commanded (no full-rotation detours)
- Wrap the reported pan angle to (−180°, 180°] in telemetry

---

## CIGI Message Reference

### Host → Simulator

| Message         | Type ID | Size | Effect                                                           |
| --------------- | ------- | ---- | ---------------------------------------------------------------- |
| `IgControl`     | 1       | 24 B | Updates IG mode and host frame counter                           |
| `EntityControl` | 2       | 48 B | Sets platform lat/lon/alt and roll/pitch/yaw; drives INS/vel_ned |
| `SensorControl` | 17      | 24 B | Commands gimbal mode, pointing, zoom, and camera (see below)     |

#### `SensorControl` Field Mapping

| Field              | Range   | Meaning                                                                                               |
| ------------------ | ------- | ----------------------------------------------------------------------------------------------------- |
| `sensor_state`     | 0–4     | Gimbal mode (see table below)                                                                         |
| `track_mode` bit 0 | 0/1     | When `sensor_state=1`: 0 → Position, 1 → Rate                                                         |
| `sensor_id`        | 0–2     | Camera: 0 = EO wide, 1 = EO narrow, 2 = IR                                                            |
| `gain`             | 0.0–1.0 | Pan target `(gain×2−1)×π` rad (Position); pan rate ×max_rate (Rate); track X offset −0.5→+0.5 (Track) |
| `level`            | 0.0–1.0 | Tilt target / rate / track Y offset (same encoding as gain)                                           |
| `ac_coupling`      | 0.0–1.0 | Zoom level (0 = wide, 1 = narrow) in Position/Rate modes; geopoint lat fraction in Geopoint mode      |
| `noise`            | 0.0–1.0 | Geopoint lon fraction (Geopoint mode only)                                                            |

#### `sensor_state` → Gimbal Mode

| `sensor_state` | `track_mode` bit 0 | Orion mode          | Behaviour                                                   |
| -------------- | ------------------ | ------------------- | ----------------------------------------------------------- |
| 0              | —                  | `OrionModeDisabled` | Motors off; telemetry still sent                            |
| 1              | 0                  | `OrionModePosition` | Slew to gain/level pan/tilt target                          |
| 1              | 1                  | `OrionModeRate`     | Continuous rate command in rad/s                            |
| 2              | —                  | `OrionModeTrack`    | Proportional track controller; gain/level = centroid offset |
| 4              | —                  | `OrionModeGeopoint` | Point at WGS84 coordinate encoded in ac_coupling/noise      |

**Geopoint coordinate encoding**:
```
target_lat_deg = ac_coupling × 180 − 90      (−90° to +90°)
target_lon_deg = noise × 360 − 180           (−180° to +180°)
```

### Simulator → Host

| Message                  | Type ID | Size | Cadence | Contents                                                    |
| ------------------------ | ------- | ---- | ------- | ----------------------------------------------------------- |
| `StartOfFrame`           | 64      | 16 B | 50 Hz   | IG mode, frame counter, timestamp                           |
| `SensorExtendedResponse` | 68      | 40 B | 10 Hz   | Gate position, sensor status, entity look-point lat/lon/alt |

#### `SensorExtendedResponse` Field Mapping

| Simulator state           | CIGI field         | Notes                                               |
| ------------------------- | ------------------ | --------------------------------------------------- |
| `pan` (rad, with jitter)  | `gate_x_pos`       | Normalised by HFOV                                  |
| `tilt` (rad, with jitter) | `gate_y_pos`       | Normalised by VFOV                                  |
| `look_lat` (rad)          | `entity_lat` (deg) | WGS84 ground look-point; 0,0,0 during camera switch |
| `look_lon` (rad)          | `entity_lon` (deg) | —                                                   |
| `look_alt` (m)            | `entity_alt` (m)   | —                                                   |
| `mode`                    | `sensor_status`    | Active→0, Tracking→1, Disabled/Fault→3              |

---

## Gimbal Modes in Detail

### Position Mode (`sensor_state=1`, `track_mode=0`)

Slews to a commanded pan/tilt using a **trapezoidal velocity profile**:

1. Accelerates from rest at `max_accel_deg_s2` up to `max_slew_rate_deg_s`
2. Cruises at max rate
3. Decelerates so it arrives at target with zero velocity
4. Overshoots are clamped to target exactly

When the commanded angle exceeds a hard-stop limit, it is silently clamped and `at_pan_limit` / `at_tilt_limit` flags are set.

### Rate Mode (`sensor_state=1`, `track_mode` bit 0 set)

Continuously integrates commanded angular rates. Gain/level map linearly to ±`max_slew_rate`. Integrates until a new command is received or mode changes. Pan is not clamped in continuous-pan configurations.

### Track Mode (`sensor_state=2`)

Applies a proportional controller that drives pan/tilt toward the commanded image centroid:

```
pan_rate  = K × track_target_x × hfov
tilt_rate = K × track_target_y × vfov
```

Track is automatically lost when the target offset exceeds 45% of the half-FOV (simulates target exiting frame). `PrimaryTrackData` in telemetry reflects active/coasting/lost status.

### Geopoint Mode (`sensor_state=4`)

Each tick, the simulator solves the inverse geolocation problem — given platform position and the commanded ground coordinate, it computes the pan/tilt angles required and feeds them into the position-mode controller. Requires a valid platform position from `EntityControl`.

---

## Geolocation

The simulator continuously computes where the gimbal is pointing on the Earth's surface using WGS84 ellipsoid math (no flat-Earth approximation):

1. Platform geodetic position → ECEF
2. Pan/tilt angles → NED line-of-sight vector → ECEF direction
3. Ray–ellipsoid intersection → look-point ECEF → geodetic
4. Look-point lat/lon/alt → `entity_lat/lon/alt` in `SensorExtendedResponse`

The `los_ecef` field in `GeolocateTelemetryCorePacket` is populated each frame. Geolocation is suppressed (returns 0,0,0) during camera switch blackout or when altitude is ≤ 0.

---

## Platform Motion & INS

`EntityControl` sets the platform position **and attitude**:

| EntityControl field | Unit       | Effect                                             |
| ------------------- | ---------- | -------------------------------------------------- |
| `lat_or_x`          | degrees    | Platform latitude                                  |
| `lon_or_y`          | degrees    | Platform longitude                                 |
| `alt_or_z`          | metres MSL | Platform altitude                                  |
| `roll`              | degrees    | Used for `gimbal_quat` / `ins_quat`                |
| `pitch`             | degrees    | —                                                  |
| `yaw`               | degrees    | Heading; offsets gimbal azimuth in LOS computation |

Computed fields written to `GeolocateTelemetryCorePacket` each frame:

| Telemetry field | Source                                                                                |
| --------------- | ------------------------------------------------------------------------------------- |
| `gimbal_quat`   | Quaternion from Rz(pan) × Ry(−tilt)                                                   |
| `ins_quat`      | ZYX Euler quaternion from platform roll/pitch/yaw; `None` during IMU dropout          |
| `vel_ned`       | NED velocity estimated from successive EntityControl positions (assumes 50 Hz update) |

---

## Fault Injection

Faults are accessible via `sim.faults` in code, or will be visible in `--diag` output:

```rust
sim.faults.inject_gps_loss();     // GPS/INS fields → 0 in telemetry
sim.faults.inject_motor_fault();  // Slew disabled; voltage sag in diagnostics
sim.faults.inject_imu_dropout();  // ins_quat → None; angular rates → 0
sim.faults.inject_thermal();      // Crown/payload temps +30 °C in diagnostics
sim.faults.clear_all();
```

With `--diag`, a one-line summary is logged at 1 Hz:
```
[DIAG] V24=23.98V  V12=12.01V  V3V3=3.30V  Crown=47.3°C  Gyro=57.1°C  Payload=41.8°C  Faults: GPS=0 Motor=0 IMU=0
```

---

## TCP Bridge (`--gimbal-ip`)

Activates transparent proxy mode: CIGI commands are translated to Orion wire protocol and forwarded to the real gimbal; the gimbal's telemetry is applied directly to simulator state before CIGI responses are sent.

```bash
./target/release/cigi_trillium --gimbal-ip 192.168.1.42
```

Data flow:
```
CIGI host  →  UDP  →  cigi_trillium  →  TCP (Orion)  →  real gimbal
CIGI host  ←  UDP  ←  cigi_trillium  ←  TCP (Orion)  ←  real gimbal
```

- Port is always 8008 on the gimbal side (Orion hardware default)
- If the TCP connection fails at startup, the process continues in simulation-only mode
- If the connection drops at runtime, automatic reconnection is attempted every 5 seconds
- Disconnect and reconnect events are printed to stderr

---

## Project Structure

```
cigi_trillium/
├── src/
│   ├── main.rs              50 Hz event loop; CLI args; bridge orchestration
│   ├── simulator/
│   │   └── mod.rs           GimbalSimulator — all physics, modes, telemetry
│   ├── geo.rs               WGS84 math (ECEF, NED, ray-cast, inverse geopoint)
│   ├── config.rs            Config struct; config.toml parser
│   ├── faults.rs            FaultState; OrionDiagnosticsPacket builder
│   ├── bridge.rs            GimbalBridge — TCP proxy to real hardware
│   ├── convert/
│   │   ├── to_orion.rs      SensorControl/EntityControl → Orion packets
│   │   └── to_cigi.rs       Orion telemetry → CIGI SensorResponse/Extended
│   ├── cigi/
│   │   ├── mod.rs
│   │   └── messages.rs      CIGI v3.3 message structs (hand-written per ICD)
│   ├── orion/
│   │   ├── mod.rs           Re-exports generated types
│   │   └── wire.rs          Orion framing: sync bytes, Fletcher-16 checksum
│   └── server/
│       └── mod.rs           CigiServer — non-blocking UDP socket
├── build.rs                 Code generator: OrionPublicProtocol.xml → Rust
├── OrionPublicProtocol.xml  Trillium Orion protocol schema (source of truth)
├── config.toml              Default runtime configuration
└── config_hd45.toml         Trillium HD45-LV-CZ-GS profile
```

---

## Tests

```bash
cargo test
```

18 tests across 4 modules:

| Module           | Test                             | Verifies                                                         |
| ---------------- | -------------------------------- | ---------------------------------------------------------------- |
| `geo`            | `ecef_roundtrip`                 | Geodetic→ECEF→geodetic within 0.01 m / 1e-10 rad                 |
| `geo`            | `look_point_nadir`               | Straight-down LOS lands directly below platform                  |
| `geo`            | `inverse_geopoint_roundtrip`     | forward + inverse geolocation agrees within 0.01 rad             |
| `cigi::messages` | `sensor_control_roundtrip`       | SensorControl encode/decode field preservation                   |
| `cigi::messages` | `start_of_frame_roundtrip`       | StartOfFrame encode/decode                                       |
| `orion::wire`    | `frame_parse_roundtrip`          | Orion wire framing + Fletcher-16 checksum                        |
| `orion::wire`    | `parse_skips_garbage`            | Sync-byte scanner skips leading junk                             |
| `orion::wire`    | `empty_payload`                  | Zero-length payload frames correctly                             |
| `orion::wire`    | `orion_cmd_packet_roundtrip`     | OrionCmdPacket encode→frame→parse→decode within scaler precision |
| `simulator`      | `slew_toward_target`             | Pan/tilt converge within 1e-4 rad in ≤ 10 s                      |
| `simulator`      | `accel_profile_smooth`           | Rate is sub-max after 1 ms (not instantaneous)                   |
| `simulator`      | `angle_limits_enforced`          | Clamping and limit flags set correctly                           |
| `simulator`      | `apply_sensor_control_sets_mode` | Mode and target mapping from SensorControl                       |
| `simulator`      | `rate_mode_integrates`           | pan = rate × dt after one tick                                   |
| `simulator`      | `camera_switch_blackout`         | entity lat/lon/alt zeroed during 200 ms switch window            |
| `simulator`      | `sensor_response_end_to_end`     | Full pipeline: command → ticks → SensorResponse                  |
| `simulator`      | `gimbal_quat_unit_length`        | gimbal_quat is a valid unit quaternion                           |
| `simulator`      | `platform_quat_unit_length`      | ins_quat is a valid unit quaternion                              |

---

## Integration Test (Python)

Sends a `SensorControl` commanding position mode, then listens for a `SensorExtendedResponse`:

```python
import socket, struct, time

HOST, PORT = '127.0.0.1', 8008

def send_sensor_control(sock, pan_frac=0.75, tilt_frac=0.5, zoom=0.0, state=1):
    """state=1 position, track_mode=0; gain/level = pan/tilt targets (0–1)."""
    pkt = bytearray(24)
    pkt[0] = 17          # type: SensorControl
    pkt[1] = 24          # size
    pkt[2] = 0           # view_id
    pkt[3] = 0           # sensor_id (camera 0)
    pkt[4] = state & 0x03
    struct.pack_into('<f', pkt,  6, pan_frac)   # gain
    struct.pack_into('<f', pkt, 10, tilt_frac)  # level
    struct.pack_into('<f', pkt, 14, zoom)       # ac_coupling = zoom
    sock.sendto(bytes(pkt), (HOST, PORT))

def send_entity_control(sock, lat=37.0, lon=-122.0, alt=1000.0, yaw=0.0):
    """Set platform position and heading."""
    pkt = bytearray(48)
    pkt[0] = 2; pkt[1] = 48
    struct.pack_into('<d', pkt, 24, lat)
    struct.pack_into('<d', pkt, 32, lon)
    struct.pack_into('<d', pkt, 40, alt)
    struct.pack_into('<f', pkt, 20, yaw)  # yaw
    sock.sendto(bytes(pkt), (HOST, PORT))

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.5)

# Set platform position 1000 m above San Francisco
send_entity_control(sock, lat=37.615, lon=-122.389, alt=1000.0, yaw=0.0)
# Command 45° pan (gain≈0.625), 10° tilt (level≈0.528), half zoom
send_sensor_control(sock, pan_frac=0.625, tilt_frac=0.528, zoom=0.5)

# Collect responses for 2 seconds
deadline = time.time() + 2.0
while time.time() < deadline:
    try:
        data, _ = sock.recvfrom(256)
        if data[0] == 68:  # SensorExtendedResponse
            lat = struct.unpack_from('<f', data, 28)[0]
            lon = struct.unpack_from('<f', data, 32)[0]
            alt = struct.unpack_from('<f', data, 36)[0]
            print(f"Look-point: lat={lat:.4f}°  lon={lon:.4f}°  alt={alt:.0f} m")
    except socket.timeout:
        pass

sock.close()
```

---

## Code Generation (`build.rs`)

At `cargo build` time, `build.rs` parses `OrionPublicProtocol.xml` (~1 700 lines, 80+ message types) and writes `$OUT_DIR/orion_generated.rs` with type-safe Rust for every packet:

| XML feature       | Generated Rust                                                                         |
| ----------------- | -------------------------------------------------------------------------------------- |
| `<enum>`          | `pub enum Foo` with `Default` + `TryFrom<u8>`                                          |
| `<struct>`        | `pub struct Foo` with `encode(&mut Vec<u8>)` / `decode(&mut &[u8])`                    |
| `<packet id="N">` | `pub struct FooPacket` with `const ID: u8 = N`, `encode() -> Vec<u8>`, `decode(&[u8])` |
| `scaler="1000"`   | `wire = (value * 1000).round() as i16`                                                 |
| `max="pi"`        | linear normalisation to `[0, MAX_INT]`                                                 |
| `bitfieldN`       | packed bits within byte, MSB-first                                                     |
| `dependsOn`       | `Option<T>` — conditionally encoded/decoded                                            |
| `variableArray`   | `Vec<T>` — length from preceding count field                                           |

To inspect the generated code:
```bash
cargo build
cat target/debug/build/cigi_trillium-*/out/orion_generated.rs | less
```
