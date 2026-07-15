// Just a blinky example to set up the environment.

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut led = Output::new(p.PIN_25, Level::Low);
    let mut n: u32 = 0;
    loop {
        led.toggle();
        info!("alive, tick {} — GPIO25 LED toggling", n);
        n = n.wrapping_add(1);
        Timer::after(Duration::from_millis(250)).await;
    }
}
