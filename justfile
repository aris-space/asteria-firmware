set shell := ["bash", "-euo", "pipefail", "-c"]

# List of all workspaces
workspaces := "fw-engine-board fw-sensor-carrier fw-sensor-carrier-v3 fw-power-board fw-communication-board fw-fuel-control-board fw-oxidizer-control-board fw-lox-valve-control-board fw-recovery-board fw-test-board crates tools"
# Workspaces to skip when running commands across all workspaces
exclude_workspaces := "hermes-can"
# Workspaces that can run tests
test_workspaces := "crates tools"
# Workspaces that have docs
doc_workspaces := "crates tools"

import 'just/artifacts.just'
import 'just/logs.just'

# Show available capabilities at repo root.
[default]
help:
    @just --list

# Format all workspaces
fmt *args:
    for ws in {{workspaces}}; do \
        case " {{exclude_workspaces}} " in *" $ws "*) continue ;; esac; \
        (cd $ws && just fmt {{args}}); \
    done

# Build all workspaces, transparently pass any args
build *args:
    for ws in {{workspaces}}; do \
        case " {{exclude_workspaces}} " in *" $ws "*) continue ;; esac; \
        (cd $ws && just build {{args}}); \
    done

# Run CI checks across all workspaces
ci-checks:
    for ws in {{workspaces}}; do \
        case " {{exclude_workspaces}} " in *" $ws "*) continue ;; esac; \
        (cd $ws && just ci-checks); \
    done

# Run host-side tests
test *args:
    for ws in {{test_workspaces}}; do \
        case " {{exclude_workspaces}} " in *" $ws "*) continue ;; esac; \
        (cd $ws && just test {{args}}); \
    done

# Build docs where available
doc *args:
    for ws in {{doc_workspaces}}; do \
        case " {{exclude_workspaces}} " in *" $ws "*) continue ;; esac; \
        (cd $ws && just doc {{args}}); \
    done

# Run clippy across all workspaces
clippy *args:
    for ws in {{workspaces}}; do \
        case " {{exclude_workspaces}} " in *" $ws "*) continue ;; esac; \
        (cd $ws && just clippy {{args}}); \
    done

# Clean rust build artifacts across all workspaces
clean:
    for ws in {{workspaces}}; do \
        case " {{exclude_workspaces}} " in *" $ws "*) continue ;; esac; \
        (cd $ws && just clean); \
    done
