use embassy_stm32::can::filter::{Action, FilterType, StandardFilter};
use embassy_stm32::can::{Can, CanConfigurator, OperatingMode};
use embassy_stm32::{bind_interrupts, peripherals};
use hermes_can::messages::Message;

use super::CanBus;

impl CanBus {
    pub fn setup(self) -> Can<'static> {
        bind_interrupts!(struct CanIrqs {
            FDCAN3_IT0 => embassy_stm32::can::IT0InterruptHandler<peripherals::FDCAN3>;
            FDCAN3_IT1 => embassy_stm32::can::IT1InterruptHandler<peripherals::FDCAN3>;
        });

        let mut can = CanConfigurator::new(self.periph, self.rx, self.tx, CanIrqs);
        can.set_bitrate(1_000_000);
        can.set_fd_data_bitrate(1_000_000, false);

        const FILTER_COUNT: usize = 28;
        let mut filters: [StandardFilter; FILTER_COUNT] = [StandardFilter {
            filter: FilterType::Disabled,
            action: Action::Disable,
        }; FILTER_COUNT];

        const _ASSERT_LEN_OK: () = {
            if Message::NUM_ENABLED_IDS >= FILTER_COUNT - 1 {
                core::panic!("Too many receiving can ids");
            }
        };
        filters[Message::NUM_ENABLED_IDS] = StandardFilter::reject_all();

        for (filter_idx, id) in Message::ENABLED_IDS.iter().enumerate() {
            filters[filter_idx] = StandardFilter {
                filter: FilterType::DedicatedSingle(*id),
                action: Action::StoreInFifo1,
            };
        }
        can.properties().set_standard_filters(&filters);

        can.start(OperatingMode::NormalOperationMode)
    }
}
