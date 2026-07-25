// Perform initial handshake with a USB device and light up the LED when connected.

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

    info!("waiting for device");
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
                continue;
            }
            _ => (),
        }

        const GET_DESC_DEVICE: [u8; 8] = [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x40, 0x00];
        let mut pdd = [0u8; 64];
        for _ in 0..100 {
            let res = bus.control_in(0, 0, 64, &GET_DESC_DEVICE, &mut pdd);
            match res {
                Ok(len) => {
                    info!(
                        "control_in returned {} bytes: {:x}",
                        len,
                        defmt::Debug2Format(&pdd[..len])
                    );
                    break;
                }
                Err(e) => {
                    info!("control_in error: {:?}", e);
                }
            }
        }
    }
}
