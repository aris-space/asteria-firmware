use data_core::can::{hal::CanDecode, sparse_decodable_can_message};

// This enum encompases all messages that the Recovery board needs to receive.
sparse_decodable_can_message! {
    enum ReceivedMessage {
        ResetAll(dp_system_management::Message::ResetAll),
        ResetSpecific(dp_system_management::Message::ResetSpecific),
        UTCTimeUpdate(dp_system_management::Message::UTCTimeUpdate),

        SteeringTargetPositions(dp_recovery_board::Message::SteeringTargetPositions),
        RecoveryPowerConfig(dp_recovery_board::Message::RecoveryPowerConfig),

        SeparationTrigger(dp_recovery_board::Message::SeparationTrigger),
        DeploymentTrigger(dp_recovery_board::Message::DeploymentTrigger),
    }
}

// This will fail to compile if the number of enabled ids exceeds the number of filters
const __ASSERT_LEN_OK: () = {
    const FILTER_COUNT: usize = 28;
    if ReceivedMessage::SUPPORTED_IDS.len() >= FILTER_COUNT - 1 {
        core::panic!("Too many receiving can ids");
    }
};

/// This serves as both an example of how to use [`ReceivedMessage`] as well as a santiy check that it works.
#[test]
fn sanity_check() {
    use data_core::can::hal::{CanDecode, CanEncode};
    let msg =
        dp_recovery_board::Message::SteeringTargetPositions(dp_recovery_board::SteeringPositions {
            left_pos: -132,
            right_pos: 32,
        });
    let mut buf = [0; 64];
    let (id, len) = msg.encode_into(&mut buf).expect("need to be serializable");
    let sparse_msg =
        ReceivedMessage::from_parts(id, &buf[..(len as usize)]).expect("neeed to decode again");

    assert!(matches!(
        sparse_msg,
        ReceivedMessage::SteeringTarget(dp_recovery_board::SteeringPositions {
            left_pos: -132,
            right_pos: 32,
        })
    ))
}
