use data_core::can::{hal::CanDecode, sparse_decodable_can_message};

sparse_decodable_can_message! {
  enum ReceivedMessage {
    ResetAll(dp_system_management::Message::ResetAll),
    ResetSpecific(dp_system_management::Message::ResetSpecific),
    UTCTimeUpdate(dp_system_management::Message::UTCTimeUpdate),
  }
}

const __ASSERT_LEN_OK: () = {
    const FILTER_COUNT: usize = 28;
    if ReceivedMessage::SUPPORTED_IDS.len() >= FILTER_COUNT - 1 {
        core::panic!("Too many receiving can ids");
    }
};
