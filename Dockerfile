FROM rust:1.85-slim AS builder
WORKDIR /app
COPY Cargo.toml Cargo.lock ./
COPY build.rs OrionPublicProtocol.xml ./
COPY src/ src/
RUN cargo build --release

FROM debian:bookworm-slim
COPY --from=builder /app/target/release/cigi_trillium /usr/local/bin/
COPY config.toml /etc/cigi_trillium/config.toml
WORKDIR /etc/cigi_trillium
EXPOSE 8008/tcp 8101/udp
ENTRYPOINT ["cigi_trillium"]
