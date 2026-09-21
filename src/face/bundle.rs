//! OSC bundle helpers, ported from oscavmgr's `src/core/bundle.rs`.

use rosc::{OscBundle, OscMessage, OscPacket, OscType};

pub const PARAM_PREFIX: &str = "/avatar/parameters/";

pub trait AvatarBundle {
    fn new_bundle() -> Self;
    fn send_parameter(&mut self, name: &str, value: OscType);
    fn send_tracking(&mut self, addr: &str, args: Vec<OscType>);
}

impl AvatarBundle for OscBundle {
    fn new_bundle() -> OscBundle {
        OscBundle {
            timetag: rosc::OscTime {
                seconds: 0,
                fractional: 0,
            },
            content: Vec::new(),
        }
    }
    fn send_parameter(&mut self, name: &str, value: OscType) {
        tracing::trace!("Sending parameter {} = {:?}", name, value);
        self.content.push(OscPacket::Message(OscMessage {
            addr: format!("{}{}", PARAM_PREFIX, name),
            args: vec![value],
        }));
    }
    fn send_tracking(&mut self, addr: &str, args: Vec<OscType>) {
        tracing::trace!("Sending tracking {} = {:?}", addr, args);
        self.content.push(OscPacket::Message(OscMessage {
            addr: addr.to_string(),
            args,
        }));
    }
}
