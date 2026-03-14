// Static platform source — writes the config-derived PlatformState once,
// then holds the channel open forever. Preserves the pre-MAVLink behavior.

use crate::config::Config;
use crate::platform::{PlatformSource, PlatformState};
use tokio::sync::watch;

pub struct StaticSource {
    state: PlatformState,
}

impl StaticSource {
    pub fn from_config(cfg: &Config) -> Self {
        Self {
            state: PlatformState::from_config(cfg),
        }
    }
}

impl PlatformSource for StaticSource {
    async fn run(self, tx: watch::Sender<PlatformState>) {
        // Send the initial state (receiver already has the seed value, but
        // send again to be explicit and to confirm the channel is healthy).
        let _ = tx.send(self.state);
        // Hold the sender open forever so the receiver never sees a closed channel.
        std::future::pending::<()>().await;
    }
}
