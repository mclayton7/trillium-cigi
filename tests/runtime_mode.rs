// Integration tests for the [runtime] mode toggle.
//
// Each test:
//   - allocates a unique pair of ports (avoids parallel-test collisions)
//   - writes a temp `config.toml` and spawns the binary with `current_dir`
//     set to that directory
//   - lets the process run briefly so the startup banner prints
//   - kills the process and asserts on captured stdout
//
// We intentionally do not assert on long-running behaviour (telemetry over
// TCP, multi-instance port-sharing) — the load-bearing coverage is the
// startup banner, which is the contract surface for "what mode is this
// process in".

use std::fs;
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicU16, Ordering};
use std::time::Duration;

const BIN: &str = env!("CARGO_BIN_EXE_cigi_trillium");

/// Per-test port allocator. Starts well above the IANA dynamic range start
/// to reduce the chance of collision with anything else on the host.
static NEXT_PORT: AtomicU16 = AtomicU16::new(31_000);

fn alloc_port_pair() -> (u16, u16) {
    let base = NEXT_PORT.fetch_add(2, Ordering::SeqCst);
    (base, base + 1)
}

struct Sandbox {
    dir: PathBuf,
}

impl Sandbox {
    fn new(name: &str) -> Self {
        let dir = std::env::temp_dir().join(format!(
            "cigi_trillium_runtime_test_{name}_{}",
            std::process::id()
        ));
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).expect("create sandbox dir");
        Self { dir }
    }

    fn write_config(&self, content: &str) {
        fs::write(self.dir.join("config.toml"), content).expect("write config.toml");
    }
}

impl Drop for Sandbox {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.dir);
    }
}

/// Run the binary briefly under `sandbox` with the given extra args, then
/// kill it and return captured (stdout, stderr).
fn run_briefly(sandbox: &Sandbox, args: &[&str], dur: Duration) -> (String, String) {
    let mut child = Command::new(BIN)
        .current_dir(&sandbox.dir)
        .args(args)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn binary");

    std::thread::sleep(dur);
    let _ = child.kill();
    let out = child.wait_with_output().expect("wait_with_output");
    (
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
    )
}

fn config_with_ports(orion_port: u16, cigi_port: u16, runtime_section: &str) -> String {
    format!(
        "{runtime_section}\
         [network]\n\
         orion_listen_port = {orion_port}\n\
         scene_generator_ip = \"127.0.0.1\"\n\
         scene_generator_cigi_port = {scene_gen}\n\
         cigi_listen_port = {cigi_port}\n",
        scene_gen = cigi_port + 100,
    )
}

#[test]
fn simulator_mode_banner_omits_cigi() {
    let (orion, cigi) = alloc_port_pair();
    let sandbox = Sandbox::new("sim_banner");
    sandbox.write_config(&config_with_ports(
        orion,
        cigi,
        "[runtime]\nmode = \"simulator\"\n",
    ));

    let (stdout, stderr) = run_briefly(&sandbox, &[], Duration::from_millis(500));

    assert!(
        stdout.contains("mode = simulator"),
        "expected simulator banner; stdout=\n{stdout}\nstderr=\n{stderr}"
    );
    assert!(
        stdout.contains("CIGI disabled"),
        "expected 'CIGI disabled' marker; stdout=\n{stdout}"
    );
    assert!(
        !stdout.contains("CIGI UDP"),
        "simulator-mode banner must not mention CIGI UDP; stdout=\n{stdout}"
    );
}

#[test]
fn bridge_mode_default_when_no_runtime_section() {
    let (orion, cigi) = alloc_port_pair();
    let sandbox = Sandbox::new("bridge_default");
    sandbox.write_config(&config_with_ports(orion, cigi, ""));

    let (stdout, stderr) = run_briefly(&sandbox, &[], Duration::from_millis(500));

    assert!(
        stdout.contains("mode = bridge"),
        "expected bridge banner; stdout=\n{stdout}\nstderr=\n{stderr}"
    );
    assert!(
        stdout.contains("CIGI UDP"),
        "bridge-mode banner must mention CIGI UDP; stdout=\n{stdout}"
    );
}

#[test]
fn cli_simulator_overrides_bridge_config() {
    let (orion, cigi) = alloc_port_pair();
    let sandbox = Sandbox::new("cli_override");
    sandbox.write_config(&config_with_ports(
        orion,
        cigi,
        "[runtime]\nmode = \"bridge\"\n",
    ));

    let (stdout, stderr) =
        run_briefly(&sandbox, &["--simulator"], Duration::from_millis(500));

    assert!(
        stdout.contains("mode = simulator"),
        "expected --simulator to override bridge config; stdout=\n{stdout}\nstderr=\n{stderr}"
    );
    assert!(
        !stdout.contains("CIGI UDP"),
        "--simulator override must skip CIGI; stdout=\n{stdout}"
    );
}

#[test]
fn cli_bridge_overrides_simulator_config() {
    let (orion, cigi) = alloc_port_pair();
    let sandbox = Sandbox::new("cli_bridge");
    sandbox.write_config(&config_with_ports(
        orion,
        cigi,
        "[runtime]\nmode = \"simulator\"\n",
    ));

    let (stdout, stderr) =
        run_briefly(&sandbox, &["--bridge"], Duration::from_millis(500));

    assert!(
        stdout.contains("mode = bridge"),
        "expected --bridge to override simulator config; stdout=\n{stdout}\nstderr=\n{stderr}"
    );
}
