{
  description = "Nix development environment for Rust";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
    rust-overlay.url = "github:oxalica/rust-overlay";
  };

  outputs = { self, nixpkgs, rust-overlay, ... }:
    let
        system = "aarch64-darwin";
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs {
            inherit system overlays;
        };
        rust-toolchain = pkgs.rust-bin.stable.latest.default.override {
            extensions = [ "rust-src" ];
            targets = [ "thumbv7em-none-eabihf" ];
        };
    in {
    devShells.${system}.default = with pkgs; mkShell {
        buildInputs =  [
            rust-toolchain
            probe-rs-tools
        ];
    };

  };
}
