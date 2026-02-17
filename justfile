workspaces := "fw-engine-board fw-sensor-carrier fw-power-board fw-communication-board fw-fuel-control-board fw-oxidizer-control-board fw-lox-valve-control-board fw-recovery-board crates tools"

# Format all workspaces
fmt:
    for ws in {{workspaces}}; do cargo fmt --manifest-path $ws/Cargo.toml --all; done
