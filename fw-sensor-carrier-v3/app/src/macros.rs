#[macro_export]
macro_rules! interrupt_executor {
    ($interrupt:ident, $prio:ident) => {{
        use embassy_stm32::interrupt;
        use embassy_stm32::interrupt::{InterruptExt, Priority};
        use embassy_executor::InterruptExecutor;

        interrupt::$interrupt.set_priority(Priority::$prio);
        static EXECUTOR: InterruptExecutor = InterruptExecutor::new();
        let spawner = EXECUTOR.start(interrupt::$interrupt);

        #[interrupt]
        #[allow(non_snake_case)]
        unsafe fn $interrupt() {
            unsafe {
                EXECUTOR.on_interrupt()
            }
        }

        spawner
    }};
}
