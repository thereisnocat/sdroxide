//! Device questions from the remote client, answered by this machine.
//!
//! The session cannot run these itself. A LAN scan takes a second and a half, a
//! connection test up to five, and a SoapySDR enumeration loads every installed
//! module and asks each to probe its bus — none of which may happen on the task
//! that is also carrying receive audio, and none of which may touch the engine
//! thread. So they run one at a time on a thread of their own, and the answer
//! goes back through the reliable lane like any other message.
//!
//! One at a time is deliberate on two counts: it puts a ceiling on what a
//! client can start here (each of these opens sockets and scans buses), and it
//! keeps the answers in the order the questions were asked, so a second Rescan
//! cannot be overtaken by the first.

use std::sync::Arc;

use sdroxide_proto::ServerMsg;
use sdroxide_types::{DeviceProbe, ProbeAnswer};
use tracing::warn;

use crate::Shared;

/// How a host answers device questions. Built by whoever starts the server —
/// the binary, which is the only place that knows how to reach each backend.
///
/// The second argument is the roster id of the radio whose session asked. Most
/// of these questions are about the machine and answer the same whoever asks;
/// the diagnostic report is about *a radio*, and a station with two of a kind
/// on it — two Icoms on the LAN is the ordinary case — has to answer with the
/// asking radio's own session and no other's.
///
/// `Fn` rather than `FnMut`: it is called from the worker thread only, but
/// nothing about a device question is stateful, and requiring `Sync` keeps it
/// shareable without a second lock.
pub type ProbeFn = Box<dyn Fn(DeviceProbe, u32) -> ProbeAnswer + Send + Sync>;

/// Requests waiting for the worker. Small on purpose: an operator pressing
/// Rescan five times while a scan runs wants the scan, not five more of them,
/// and a client that floods this lane is dropped rather than queued. Deep
/// enough for the handful a settings dialog asks the moment it opens, and no
/// deeper.
const QUEUE: usize = 8;

/// One worker for the station, not one per radio: a bus scan is a question
/// about this machine, and two radios asking at once is exactly the pile-up
/// the single worker exists to prevent. Which radio asked rides with the
/// question, so the answer goes back to the session that wanted it.
pub(crate) struct ProbeHub {
    tx: Option<crossbeam_channel::Sender<(Arc<Shared>, DeviceProbe)>>,
}

impl ProbeHub {
    /// Start the worker, or make one that refuses everything when this server
    /// was built without a way to answer.
    pub(crate) fn start(probe: Option<ProbeFn>) -> ProbeHub {
        let Some(probe) = probe else { return ProbeHub { tx: None } };
        let (tx, rx) = crossbeam_channel::bounded::<(Arc<Shared>, DeviceProbe)>(QUEUE);
        std::thread::Builder::new()
            .name("sdroxide-probe".into())
            .spawn(move || {
                for (shared, req) in rx {
                    let answer = probe(req, shared.id);
                    reply(&shared, answer);
                }
            })
            .expect("spawn probe thread");
        ProbeHub { tx: Some(tx) }
    }
}

/// Take a question from the session. Answers straight away where this server
/// has no prober, so the client can grey the controls out with a reason instead
/// of waiting for something that will never arrive.
pub(crate) fn ask(shared: &Arc<Shared>, req: DeviceProbe) {
    match shared.probes.tx.as_ref() {
        Some(tx) => {
            if tx.try_send((shared.clone(), req)).is_err() {
                warn!("device probe dropped: the queue is full");
            }
        }
        None => reply(shared, ProbeAnswer::Unsupported),
    }
}

/// Into the reliable lane of whichever session is attached — which may no
/// longer be the one that asked, if it dropped mid-scan. Harmless: an answer
/// nobody is waiting for lands in a list that is redrawn on demand.
fn reply(shared: &Arc<Shared>, answer: ProbeAnswer) {
    if let Some(s) = shared.session.lock().unwrap().as_ref() {
        let _ = s.reliable.try_send(ServerMsg::ProbeAnswer(Box::new(answer)));
    }
}
