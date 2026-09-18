//! Send an AutoBoneProcessRequest over the SolarXR Unix socket.
//!
//! ```text
//! cargo run --example send_autobone [record|process] [/run/user/1000/SlimeVRRpc]
//! ```

use solarxr_protocol::flatbuffers::{self, FlatBufferBuilder};
use solarxr_protocol::rpc::{
    AutoBoneProcessRequest, AutoBoneProcessRequestArgs, AutoBoneProcessType, RpcMessage,
    RpcMessageHeader, RpcMessageHeaderArgs,
};
use solarxr_protocol::{MessageBundle, MessageBundleArgs};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::UnixStream;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let action = args.get(1).map(|s| s.as_str()).unwrap_or("record");
    let path = args
        .get(2)
        .cloned()
        .unwrap_or_else(|| "/run/user/1000/SlimeVRRpc".to_string());

    let ptype = match action {
        "record" => AutoBoneProcessType::RECORD,
        "process" => AutoBoneProcessType::PROCESS,
        "apply" => AutoBoneProcessType::APPLY,
        _ => AutoBoneProcessType::NONE,
    };

    let mut stream = UnixStream::connect(&path).await?;
    eprintln!("connected to {path}; sending {action}");

    let mut fbb = FlatBufferBuilder::new();
    let req = AutoBoneProcessRequest::create(
        &mut fbb,
        &AutoBoneProcessRequestArgs {
            process_type: ptype,
        },
    );
    let header = RpcMessageHeader::create(
        &mut fbb,
        &RpcMessageHeaderArgs {
            tx_id: None,
            message_type: RpcMessage::AutoBoneProcessRequest,
            message: Some(flatbuffers::WIPOffset::new(req.value())),
        },
    );
    let msgs = fbb.create_vector(&[header]);
    let bundle = MessageBundle::create(
        &mut fbb,
        &MessageBundleArgs {
            rpc_msgs: Some(msgs),
            ..Default::default()
        },
    );
    fbb.finish(bundle, None);

    // 4-byte little-endian length prefix + payload.
    let body = fbb.finished_data();
    let len = (body.len() + 4) as u32;
    stream.write_all(&len.to_le_bytes()).await?;
    stream.write_all(body).await?;

    // Read and report the status response.
    let mut len_buf = [0u8; 4];
    if stream.read_exact(&mut len_buf).await.is_ok() {
        let rlen = u32::from_le_bytes(len_buf) as usize;
        if (4..=1024 * 1024).contains(&rlen) {
            let mut resp = vec![0u8; rlen - 4];
            if stream.read_exact(&mut resp).await.is_ok() {
                if let Ok(bundle) = flatbuffers::root::<MessageBundle>(&resp) {
                    if let Some(msgs) = bundle.rpc_msgs() {
                        for i in 0..msgs.len() {
                            let h = msgs.get(i);
                            eprintln!("response message_type = {:?}", h.message_type());
                        }
                    }
                }
            }
        }
    }
    Ok(())
}
