FROM rust:1.83 AS builder

RUN USER=root cargo new --bin app
WORKDIR /app

COPY Cargo.toml Cargo.lock ./
RUN cargo build --release
RUN rm src/*.rs
COPY ./src ./src

RUN cargo build --release

FROM debian:bookworm-slim
RUN apt-get update & apt-get install -y extra-runtime-dependencies & rm -rf /var/lib/apt/lists/*

RUN groupadd -r auklet && useradd -r -g auklet auklet
RUN mkdir -p /usr/local/bin && \
    chown auklet:auklet /usr/local/bin
COPY --from=builder /usr/local/cargo/bin/auklet /usr/local/bin/auklet
ENTRYPOINT "/usr/local/bin/auklet"
