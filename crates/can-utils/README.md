# can-utils

This crate is built very specifically towards a pattern of IO that is in the authors opinion particularly clean.
The following will attempt to explain this pattern.

## Basic pattern

The idea is that most control and signaling data (thus over CAN to the flight computer),
is based around `Watch`s, which hold the most recent value.
This includes inputs and outputs, thus event messages don't follow this pattern.
The inner types should be identical to the type sent over the CAN bus,
which allows a very simple mapping between values and messages.

Embassy tasks should then read data from the output watches and forward it to the `CanTx`.
Another task reads from `CanRx` and stores into the appropriate input watches.
For reading and writing, the `rxtx` module provides nice typed abstractions.

In order to configure hardware receive filters and have less overhead,
a board specific enum can be defined to contain only a subset of the existing messages.
For transmit, the normal messages can be used though.

See also [the wiki on the communication framework](https://wiki.aris-space.ch/en/rocketry/teams/ASTERIA/software/communication-framework)
for how the message definition fit into the picture.

## Code examples

To make it more concrete,
the firmware should define a static struct each with all relevant input and all relevant output data,
for example:

```rust
use can_utils::broadcast::Broadcast;
use can_utils::collector::Collector;
// [some other uses]

#[derive(Collector)]
#[collector(
    message_type = "ReceivedMessage", // defined below
    update_expr = "#field.sender().send(#value);" // common for `Watch`
)]
pub struct Inputs {
    /// watch for giving steering target positions to steering_task
    #[collector(pattern = "ReceivedMessage::SteeringTargetPositions(#value)")]
    pub steering_target_positions: Watch<CriticalSectionRawMutex, SteeringPositions, 3>,

    /// watch for setting steering power
    #[collector(pattern = "ReceivedMessage::RecoveryPowerConfig(#value)")]
    pub steering_power: Watch<CriticalSectionRawMutex, bool, 1>,
}

pub static INPUTS: Inputs = Inputs {
    steering_target_positions: Watch::new(),
    steering_power: Watch::new(),
};

#[derive(Broadcast)]
#[broadcast(loop_type = "can_utils::broadcast::ResponsiveLoop")]
pub struct Outputs {
    /// Position data read from the motors
    #[broadcast(
        filter_map = "#value.map(dp_recovery_board::Message::SteeringActualPositions)",
        min_freq_hz = 0.1,
        max_freq_hz = 15.
    )]
    pub steering_actual_positions: Watch<CriticalSectionRawMutex, Option<SteeringPositions>, 2>,
}

pub static OUTPUTS: Outputs = Outputs {
    steering_actual_positions: Watch::new(),
};
```

One can note here the `Broadcast` and `Collector` derives,
which actually do exactly the given easy mapping between values and messages.

The needed subset of messages is what `ReceivedMessage` defines, using something like

```rust
// This enum encompasses all messages that the Recovery board needs to receive.
data_core::can::sparse_decodable_can_message! {
    enum ReceivedMessage {
        ResetAll(dp_system_management::Message::ResetAll),
        ResetSpecific(dp_system_management::Message::ResetSpecific),

        SteeringTargetPositions(dp_recovery_board::Message::SteeringTargetPositions),
        RecoveryPowerConfig(dp_recovery_board::Message::RecoveryPowerConfig),

        [...]
    }
}
```

Then you can setup CAN, receive messages and spawn tasks to send updates like this:

```rust
use can_utils::rxtx::{TypedCanReceive as _, TypedCanTransmit};
use can_utils::setup::{make_multiplexable, setup_can};
// [some other uses]

let can = setup_can(todo!(), todo!(), todo!(), Irqs, ReceivedMessage::SUPPORTED_IDS);
let (can_tx, mut can_rx, _prop) = can.split();
let can_tx = make_multiplexable(can_tx).await;

OUTPUTS.start_broadcasting(spawner, can_tx).unwrap();

loop {
    // This uses the `TypedCanReceive` trait
    match can_rx.recv().await {
        Ok(ReceivedMessage::ResetAll(_)) => {
            info!("Resetting Recovery Board");
            cortex_m::peripheral::SCB::sys_reset();
        }
        Ok(ReceivedMessage::ResetSpecific(board)) => {
            if board == THIS_BOARD_ID {
                info!("Resetting Recovery Board");
                cortex_m::peripheral::SCB::sys_reset();
            }
        }
        Ok(msg) => {
            // update the collected inputs and ignore if the message is irrelevant
            let _ = INPUTS.update_from(msg);
        }
        Err(e) => {
            error!("Can bus error {}", e);
            Timer::after_millis(10).await;
        }
    }
}
```

Other messages can be sent using the `TypedCanTransmit`:

```rust
let mut tx = can_tx.lock().await;
match with_timeout(
    CAN_TX_TIMEOUT,
    tx.transmit(dp_recovery_board::Message::BoardStatus(msg)),
)
.await
{
    Ok(Ok(_)) => {
        trace!("sent Board status message");
    }
    Ok(Err(err)) => {
        error!("CAN TX error: {:?}", err);
    }
    Err(_) => {
        error!("CAN TX timed out after {} ms", CAN_TX_TIMEOUT);
    }
}
```
