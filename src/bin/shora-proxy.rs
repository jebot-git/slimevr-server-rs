//! Persistent WiVRn socket owner; the tracking backend can restart independently.
#[path = "../proxy.rs"]
mod proxy;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    use clap::Parser;
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::try_from_default_env()
            .unwrap_or_else(|_| "info".into()))
        .init();
    proxy::run(proxy::Options::parse()).await
}
