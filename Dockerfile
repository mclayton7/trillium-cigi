# trillium-cigi — multi-stage Rust build
FROM rust:1.85-bookworm AS builder

WORKDIR /build

# Cache dependency compilation
COPY Cargo.toml Cargo.lock ./
RUN mkdir src && echo 'fn main(){}' > src/main.rs && cargo build --release && rm -rf src

# Full build
COPY . .
RUN touch src/main.rs && cargo build --release

# ── Runtime ───────────────────────────────────────────────────────────────
FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    netcat-openbsd \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd -g 1000 trillium && useradd -u 1000 -g trillium -m -s /bin/bash trillium

COPY --from=builder /build/target/release/cigi_trillium /opt/trillium-cigi/cigi_trillium
COPY config.toml /opt/trillium-cigi/config.toml
COPY OrionPublicProtocol.xml /opt/trillium-cigi/OrionPublicProtocol.xml

RUN chmod +x /opt/trillium-cigi/cigi_trillium \
    && chown -R trillium:trillium /opt/trillium-cigi

USER trillium
WORKDIR /opt/trillium-cigi

ENTRYPOINT ["/opt/trillium-cigi/cigi_trillium"]
