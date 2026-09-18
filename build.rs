fn main() {
    println!("cargo:rerun-if-changed=proto/messages.proto");
    std::env::set_var("PROTOC", protoc_bin_vendored::protoc_bin_path().unwrap());
    prost_build::compile_protos(&["proto/messages.proto"], &["proto/"]).unwrap();
}
