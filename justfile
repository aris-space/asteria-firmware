workspaces := "fw-engine-board fw-sensor-carrier fw-power-board fw-communication-board fw-fuel-control-board fw-oxidizer-control-board fw-lox-valve-control-board fw-recovery-board crates tools"

# Format all workspaces
fmt:
    for ws in {{workspaces}}; do (cd $ws && just fmt); done

# Build all workspaces, transparently pass any args
build *args:
    for ws in {{workspaces}}; do (cd $ws && just build {{args}}); done

# Run CI checks across all workspaces
ci-checks:
    for ws in {{workspaces}}; do (cd $ws && just ci-checks); done
