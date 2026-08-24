//! Errors for the transport and device layers.

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// A LAN connect or read/write failure. Carries the actionable sentence,
    /// not the raw `io::Error` — a wrong address, a firewalled port and a
    /// radio that closed the connection all want different next steps from
    /// an operator.
    #[error("{0}")]
    Net(String),

    /// [`crate::device::Device::service`]'s retry budget ran out — the radio
    /// never acknowledged a command after
    /// [`crate::device::Device::MAX_ATTEMPTS`] sends.
    #[error("no acknowledgement for command 0x{instruction:02X} after {attempts} attempt(s)")]
    NoAck { instruction: u8, attempts: u32 },

    /// [`crate::device::Transport::send_command`] returned `false`.
    #[error("the transport rejected a command")]
    CommandRejected,

    /// [`crate::device::Device::set_hardware_diversity_from`]: the ratio
    /// needs more than the radio's 0.001..8x expressible range.
    #[error("the ratio needs more than the radio's 0.001..8x range — try swapping the channels")]
    NotRepresentable,
}
