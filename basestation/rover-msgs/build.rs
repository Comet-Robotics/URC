use std::io::Result;
fn main() -> Result<()> {
    let mut config = prost_build::Config::new();
    config.compile_protos(&["src/msgs.proto"], &["src/"])?;
    Ok(())
}