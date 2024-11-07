use std::process::Command;

fn main() {
    // Source the ROS2 environment
    let output = Command::new("bash")
        .arg("-c")
        .arg("source /opt/ros/humble/setup.bash && echo $AMENT_PREFIX_PATH")
        .output()
        .expect("Failed to source ROS2 environment");

    let ament_prefix_path = String::from_utf8_lossy(&output.stdout);

    println!("cargo:rustc-env=AMENT_PREFIX_PATH={}", ament_prefix_path);
    println!("cargo:rerun-if-changed=build.rs");
}
