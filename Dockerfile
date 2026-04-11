# trillium-cigi — multi-stage Rust build.
#
# IMPORTANT: this Dockerfile expects a build context of the **parent
# directory** of both `trillium-cigi/` and `sim-environment/` (i.e.
# /opt/mac/sim-environment/ on the host). That's what lets Docker see
# both this crate AND the sibling sim-environment/sim-core/ it depends
# on via path.
#
# The sim-environment/docker-compose.yml sets this up:
#   context: ..
#   dockerfile: trillium-cigi/Dockerfile
#
# trillium-cigi is a **standalone Cargo project**, not a workspace
# member — cargo rejects out-of-tree workspace members and the crate
# lives outside sim-environment/'s directory. The build runs as a plain
# `cargo build --release` from inside the crate directory; sim-core is
# resolved via the relative path dep declared in trillium-cigi/Cargo.toml.

FROM rust:1.85-bookworm AS builder

WORKDIR /build

# ── Dependency cache layer ────────────────────────────────────────────────
# Mirror the minimum path structure the crate expects:
#   /build/
#   ├── sim-environment/sim-core/    (sibling dep via ../sim-environment/sim-core)
#   └── trillium-cigi/               (WORKDIR for the real build)
#
# Manifest + build.rs + OrionPublicProtocol.xml only (no real source) so
# cargo can resolve and compile the dependency graph. The layer caches on
# Cargo.toml changes.
COPY sim-environment/sim-core/Cargo.toml sim-environment/sim-core/Cargo.toml
COPY trillium-cigi/Cargo.toml trillium-cigi/Cargo.toml
COPY trillium-cigi/build.rs trillium-cigi/build.rs
COPY trillium-cigi/OrionPublicProtocol.xml trillium-cigi/OrionPublicProtocol.xml
RUN mkdir -p sim-environment/sim-core/src trillium-cigi/src \
    && echo 'pub fn noop() {}' > sim-environment/sim-core/src/lib.rs \
    && echo 'fn main() {}' > trillium-cigi/src/main.rs \
    && (cd trillium-cigi && cargo build --release) \
    && rm -rf sim-environment/sim-core/src trillium-cigi/src

# ── Full build ────────────────────────────────────────────────────────────
# Copy the real source tree. The trillium-cigi crate has src/ + config.toml;
# sim-core has src/ + Cargo.toml. Touch the entrypoints so cargo rebuilds
# the crates while reusing cached dependencies.
COPY sim-environment/sim-core sim-environment/sim-core
COPY trillium-cigi trillium-cigi
RUN touch trillium-cigi/src/main.rs sim-environment/sim-core/src/lib.rs \
    && cd trillium-cigi && cargo build --release

# ── Runtime ───────────────────────────────────────────────────────────────
FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    netcat-openbsd \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd -g 1000 trillium && useradd -u 1000 -g trillium -m -s /bin/bash trillium

COPY --from=builder /build/trillium-cigi/target/release/cigi_trillium /opt/trillium-cigi/cigi_trillium
COPY trillium-cigi/config.toml /opt/trillium-cigi/config.toml
COPY trillium-cigi/OrionPublicProtocol.xml /opt/trillium-cigi/OrionPublicProtocol.xml

RUN chmod +x /opt/trillium-cigi/cigi_trillium \
    && chown -R trillium:trillium /opt/trillium-cigi

USER trillium
WORKDIR /opt/trillium-cigi

# sim-environment orchestrator Phase 6 healthcheck. Probe the Orion
# TCP listener on :8008 with netcat (installed in the runtime image
# above). 10s interval matches the orchestrator's HealthMonitor tick
# cadence so /status reflects trillium state within one cycle.
HEALTHCHECK --interval=10s --timeout=3s --start-period=15s --retries=3 \
    CMD nc -z 127.0.0.1 8008 || exit 1

ENTRYPOINT ["/opt/trillium-cigi/cigi_trillium"]
