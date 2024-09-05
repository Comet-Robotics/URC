{
  inputs = {
    # This is for the unstable nixpkgs branch
    nixpkgs-unstable.url = "github:nixos/nixpkgs/nixos-unstable";
    
    # This is for the nixpkgs that nix-ros-overlay follows
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.url = "github:nixos/nixpkgs";  # Let nix-ros-overlay choose its version
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };

  outputs = { self, nix-ros-overlay, nixpkgs, nixpkgs-unstable, home-manager }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        # Use nixpkgs-unstable for non-ROS packages
        unstablePkgs = import nixpkgs-unstable {
          inherit system;
        };
        # Use the nixpkgs version from nix-ros-overlay for ROS packages
        rosPkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
        # opencvWithGTK = unstablePkgs.opencv4.override {
        #   enableGtk2 = true;
        #   inherit (unstablePkgs) gtk2;
        # };

        # Python 3.11 with OpenCV
        pythonWithOpenCV = unstablePkgs.python311.withPackages (ps: [
          # opencvWithGTK
        ]);
      in {
        devShells.default = unstablePkgs.mkShell {
          name = "URC Dev Environment";
          shellHook = ''
            export LD_LIBRARY_PATH=${unstablePkgs.librealsense}/lib:$LD_LIBRARY_PATH; . ./install/local_setup.sh;
          '';
          packages = [
            # Non-ROS packages from unstable nixpkgs
            rosPkgs.colcon
            unstablePkgs.librealsense
            unstablePkgs.v4l-utils

            # Python 3.11 with OpenCV
            pythonWithOpenCV

            # ROS packages from nix-ros-overlay
            (with rosPkgs.rosPackages.humble; buildEnv {
              paths = [
                ros-core
                ament-cmake
                ament-cmake-core
                sensor-msgs
                std-msgs
                rosidl-default-generators
                builtin-interfaces
                rosidl-default-runtime
                python-cmake-module
                cv-bridge
                image-transport
                rclcpp
                rclcpp-components
                geometry-msgs
                nav-msgs
                tf2
                tf2-ros
                launch-ros
                diagnostic-updater
                tf2-eigen
              ];
            })
          ];
        };
      });

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
