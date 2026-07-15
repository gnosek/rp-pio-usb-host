// Report plug/unplug events of a USB device connected to the RP2040 PIO USB host bus.

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::InterruptHandler;
use embassy_usb_driver::host::DeviceEvent;
use rp_pio_usb_host::bus::Pulldown;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut led = Output::new(p.PIN_25, Level::Low);

    let mut bus =
        rp_pio_usb_host::bus::Bus::new(p.PIO0, p.PIN_0, p.PIN_1, Irqs, Pulldown::External);
    loop {
        let event = bus.wait_for_device_event().await;
        info!("device event: {:?}", event);

        match event {
            DeviceEvent::Connected(_) => {
                led.set_high();
            }
            DeviceEvent::Disconnected => {
                led.set_low();
            }
            _ => (),
        }
    }
}
