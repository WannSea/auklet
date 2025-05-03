FROM lukemathwalker/cargo-chef:latest as chef
WORKDIR /app

FROM chef AS planner
COPY ./Cargo.toml ./Cargo.lock ./
COPY ./src ./src
RUN cargo chef prepare

FROM chef AS builder
RUN apt-get update && apt-get install libudev-dev -y && rm -rf /var/lib/apt/lists/*
COPY --from=planner /app/recipe.json .
RUN cargo chef cook --release
COPY . .
RUN cargo build --release
RUN mv ./target/release/auklet ./app

FROM debian:stable-slim AS runtime
RUN apt-get update && apt-get install libudev-dev -y && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY --from=builder /app/app /usr/local/bin/auklet
ENTRYPOINT ["/usr/local/bin/auklet"]
