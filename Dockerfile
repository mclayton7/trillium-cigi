# trillium-cigi — multi-stage Rust build.
#
# IMPORTANT: this Dockerfile expects a build context of the **workspace root**
# (the directory containing Cargo.toml and all member crates), not the
# trillium-cigi/ subdirectory. docker-compose.yml sets `context: .` and
# `dockerfile: trillium-cigi/Dockerfile` for that reason.

FROM rust:1.85-bookworm AS builder

WORKDIR /build

# ── Dependency cache layer ────────────────────────────────────────────────
# Copy ONLY the Cargo manifests (workspace root + every member crate) and
# create empty src/main.rs + src/lib.rs stubs so cargo can resolve and
# compile the dependency graph without needing any real source. This layer
# gets cached as long as no Cargo.toml changes, which is the common case.
COPY Cargo.toml Cargo.lock ./
COPY sim-core/Cargo.toml sim-core/Cargo.toml
COPY trillium-cigi/Cargo.toml trillium-cigi/Cargo.toml
COPY trillium-cigi/build.rs trillium-cigi/build.rs
COPY trillium-cigi/OrionPublicProtocol.xml trillium-cigi/OrionPublicProtocol.xml
COPY platform-dis-bridge/Cargo.toml platform-dis-bridge/Cargo.toml
RUN mkdir -p sim-core/src trillium-cigi/src platform-dis-bridge/src \
    && echo 'pub fn noop() {}' > sim-core/src/lib.rs \
    && echo 'fn main() {}' > trillium-cigi/src/main.rs \
    && echo 'fn main() {}' > platform-dis-bridge/src/main.rs \
    && cargo build --release -p cigi_trillium \
    && rm -rf sim-core/src trillium-cigi/src platform-dis-bridge/src

# ── Full build ────────────────────────────────────────────────────────────
# Copy the real source tree. Touching the main.rs files forces cargo to
# rebuild the member crates while reusing the cached dependency objects.
COPY sim-core sim-core
COPY trillium-cigi trillium-cigi
COPY platform-dis-bridge platform-dis-bridge
RUN touch trillium-cigi/src/main.rs platform-dis-bridge/src/main.rs sim-core/src/lib.rs \
    && cargo build --release -p cigi_trillium

# ── Runtime ───────────────────────────────────────────────────────────────
FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    netcat-openbsd \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd -g 1000 trillium && useradd -u 1000 -g trillium -m -s /bin/bash trillium

COPY --from=builder /build/target/release/cigi_trillium /opt/trillium-cigi/cigi_trillium
COPY trillium-cigi/config.toml /opt/trillium-cigi/config.toml
COPY trillium-cigi/OrionPublicProtocol.xml /opt/trillium-cigi/OrionPublicProtocol.xml

RUN chmod +x /opt/trillium-cigi/cigi_trillium \
    && chown -R trillium:trillium /opt/trillium-cigi

USER trillium
WORKDIR /opt/trillium-cigi

ENTRYPOINT ["/opt/trillium-cigi/cigi_trillium"]
