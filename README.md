# RP2040/RP235x PIO-based USB host driver

This crate implements a USB host driver for the RP2040 and RP235x microcontrollers using the PIO (Programmable
Input/Output) feature. It allows these microcontrollers to act as USB hosts on arbitrary GPIO pins, without using
dedicated USB hardware.

The driver in this crate implements the [USBController]((https://docs.embassy.dev/embassy-usb-driver/0.2.2/default/host/trait.UsbHostController.html))
trait from the [embassy-usb-driver](https://docs.embassy.dev/embassy-usb-driver/0.2.2/default/index.html) crate,
which is part of the Embassy project. This lets you use the higher level abstractions from
[embassy-usb-host](https://docs.embassy.dev/embassy-usb-host/git/default/index.html) to interact with various
classes of USB devices.

Usage outside the Embassy ecosystem is not supported yet but should be fairly straightforward to implement. If you
want to use this crate outside of Embassy, please open an issue.

It supports USB 1.1 Low-Speed (1.5Mbps) and Full-Speed (12Mbps) devices. Hubs are supported by `embassy-usb-host`,
but low-speed devices behind a full-speed hub are *not* supported (yet). High-speed devices (480Mbps) and faster
are not supported and won't be.

This crate is heavily inspired by [Pico-PIO-USB](https://github.com/sekigon-gonnoc/Pico-PIO-USB), especially for
the PIO programs that were lifted directly from that project. All credit for the PIO programs goes to the original
authors.

## Required resources

To add a USB host to your project, you will need:
* A RP2040 or RP235x microcontroller. **Note**: tested with Raspberry Pi Pico, and Pico 2. Other boards should work but
  are untested, in particular RP235xB-based boards (with over 32 GPIOs) were not tested. If you do have a RP235xB board,
  I'd appreciate feedback (both positive and negative).
* Two **consecutive** GPIO pins for the USB D+ and D- lines. Tested in the "D+ is the lower pin" configuration, but
  the other way around should work too. Please file an issue if it does not.
* One full PIO block dedicated to the USB host driver. The driver uses 3 state machines and all 32 instructions 
  of the PIO program memory.
* At least 96 MHz of system clock speed. The USB host driver determines the PIO clock divisor automatically based
  on the system clock speed, but a frequency that does not need too many fractional bits to get to 96 MHz is preferred. 
  120 MHz is a good value, 121 MHz probably not so much.

## Wiring

Each USB data line should have a 22 Ω series resistor near the RP GPIO and a 15 kΩ pull-down resistor to ground
on the connector side.

```text
5V --------------------------------- USB connector VBUS

RP GPIO (D+) ----[ 22 Ω ]------+---- USB connector D+
                               |
                            [ 15 kΩ ]
                               |
                              GND

RP GPIO (D-) ----[ 22 Ω ]------+---- USB connector D-
                               |
                            [ 15 kΩ ]
                               |
                              GND

GND -------------------------------- USB connector GND
```

You may get away with simply wiring the D+ and D- lines directly to the RP GPIOs, but this is not recommended.
The series resistors help reduce reflections on the USB lines, and the pull-down resistors help ensure
that the USB device is properly detected when it is connected. If you do not use the external pulldowns, you will need
to enable the internal pull-downs on the RP GPIOs when creating the USB host instance.

## Usage

To use this crate, add it as a dependency in your `Cargo.toml`:

```toml
[dependencies]
embassy-usb-host = "0.2.2"
rp-pio-usb-host = { version = "0.1.0", features = ["rp2040"] }
static_cell = { version = "2.1.1"}
portable-atomic = { version = "1", default-features = false, features = ["critical-section"] }
```

You must enable exactly one of the features `rp2040`, `rp235xa` or `rp235xb` depending on which microcontroller
you are using.

Then, in your code, you can create a USB host instance and use it to interact with USB devices:

```rust
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_usb_host::BusState;
use rp_pio_usb_host::embassy::Bus;
use static_cell::StaticCell;

// Bind IRQ0 of the chosen PIO block to the PIO interrupt handler.
// If you want to use a different PIO block, change `PIO0` to `PIO1` or `PIO2` (for RP235x) everywhere.
bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

// The idle task handles USB keepalives that prevent the USB device from suspending due to bus inactivity.
// It wakes up every 1ms. The timing is less critical for LS devices (the suspend deadline is 3ms), but for FS devices,
// the USB spec requires that the host send a keepalive every 1ms. If you do lots of blocking work in your main task,
// you may want to spawn this idle task on a separate interrupt-based executor to ensure that it runs on time.
#[embassy_executor::task]
async fn usb_idle_task(bus: &'static Bus<'static, PIO0>) {
    bus.idle_task().await;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Create a static cell to hold the USB bus instance and initialize it.
    static BUS: StaticCell<Bus<PIO0>> = StaticCell::new();
    let bus = BUS.init(Bus::new(
        p.PIO0,  // use PIO0 for the USB host
        p.PIN_0, // D+ line on GPIO0 (physical pin 1 on a RPi Pico)
        p.PIN_1, // D- line on GPIO1 (physical pin 2 on a RPi Pico)
        Irqs,
        rp_pio_usb_host::bus::Pulldown::External, // `Internal` if you don't have external pulldown resistors on the USB lines
    ));
    
    // Spawn the USB idle task to handle keepalives.
    spawner.spawn(usb_idle_task(bus).unwrap());
    
    // Wrap the hardware bus in an embassy_usb_host bus controller and bus state.
    let controller = bus.controller();
    static BUS_STATE: BusState = BusState::new();
    let (mut bus_ctrl, bus) = embassy_usb_host::bus(controller, &BUS_STATE);
  
    // Now you can use `bus_ctrl` and `bus` to interact with USB devices.
}
```
