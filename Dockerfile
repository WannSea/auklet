FROM rust:bookworm AS builder
WORKDIR /usr/src/auklet
COPY . .
RUN apt-get update
RUN apt-get install -y libudev-dev
RUN cargo install --path .


FROM debian:bookworm-slim
RUN apt-get update & apt-get install -y extra-runtime-dependencies & rm -rf /var/lib/apt/lists/*
COPY --from=builder /usr/local/cargo/bin/auklet /usr/local/bin/auklet
ENTRYPOINT "/usr/local/bin/auklet"