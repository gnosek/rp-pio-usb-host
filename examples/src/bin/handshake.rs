// Perform initial handshake with a USB device and light up the LED when connected.

#![no_std]
#![no_main]

use defmt::{Debug2Format, debug, error, info};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::InterruptHandler;
use embassy_usb_driver::host::{DeviceEvent, PipeError};
use embassy_usb_host::descriptor::{DeviceDescriptor, DeviceDescriptorPartial, USBDescriptor};
use rp_pio_usb_host::bus::Pulldown;
use rp_pio_usb_host::pio_instance::UsbPioInstance;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

enum DescriptorResponse {
    Full(DeviceDescriptor),
    Partial(DeviceDescriptorPartial),
    DecodeError,
}

fn get_device_descriptor<PIO: UsbPioInstance>(
    bus: &mut rp_pio_usb_host::bus::Bus<PIO>,
    mps: Option<u16>,
) -> Result<DescriptorResponse, PipeError> {
    let get_device =
        embassy_usb_host::control::SetupPacket::get_device_descriptor(mps.unwrap_or(8));
    let mut pdd = [0u8; 64];

    let len = bus.control_in(0, 0, mps.unwrap_or(8), &get_device.to_bytes(), &mut pdd)?;

    if mps.is_some() {
        let desc = match DeviceDescriptor::try_from_bytes(&pdd[..len]) {
            Ok(desc) => desc,
            Err(e) => {
                error!(
                    "failed to parse full device descriptor {:x}: {}",
                    Debug2Format(&pdd[..len]),
                    e
                );
                return Ok(DescriptorResponse::DecodeError);
            }
        };
        Ok(DescriptorResponse::Full(desc))
    } else {
        let desc = match DeviceDescriptorPartial::try_from_bytes(&pdd[..len]) {
            Ok(desc) => desc,
            Err(e) => {
                error!(
                    "failed to parse partial device descriptor {:x}: {}",
                    Debug2Format(&pdd[..len]),
                    e
                );
                return Ok(DescriptorResponse::DecodeError);
            }
        };
        Ok(DescriptorResponse::Partial(desc))
    }
}

fn show_device_descriptor<PIO: UsbPioInstance>(
    bus: &mut rp_pio_usb_host::bus::Bus<PIO>,
) -> Result<u16, PipeError> {
    let mut mps = None;
    for _ in 0..100 {
        match get_device_descriptor(bus, mps) {
            Ok(DescriptorResponse::Full(desc)) => {
                debug!("full device descriptor: {:?}", Debug2Format(&desc));
                let usb_major_version = desc.bcd_usb >> 8;
                let usb_minor_version = desc.bcd_usb & 0xff;
                let device_major_version = desc.bcd_device >> 8;
                let device_minor_version = desc.bcd_device >> 4 & 0xf;
                let device_patch_version = desc.bcd_device & 0xf;
                info!(
                    "connected device: VID={:04x}, PID={:04x}, USB version {}.{}, device version {}.{}.{} , max_packet_size0={}",
                    desc.vendor_id,
                    desc.product_id,
                    usb_major_version,
                    usb_minor_version,
                    device_major_version,
                    device_minor_version,
                    device_patch_version,
                    desc.max_packet_size0
                );
                return Ok(desc.max_packet_size0 as u16);
            }
            Ok(DescriptorResponse::Partial(desc)) => {
                debug!("partial device descriptor: {:?}", Debug2Format(&desc));
                mps = Some(desc.max_packet_size0 as u16);
            }
            Ok(DescriptorResponse::DecodeError) => {
                error!("failed to decode device descriptor");
                return Err(PipeError::Stall);
            }
            Err(e) => {
                error!("error getting device descriptor: {:?}", e);
                return Err(e);
            }
        }
    }
    Err(PipeError::Stall)
}

fn get_device_config<PIO: UsbPioInstance>(
    bus: &mut rp_pio_usb_host::bus::Bus<PIO>,
    mps: u16,
) -> Result<u16, PipeError> {
    debug!("getting initial configuration descriptor",);
    let get_config = embassy_usb_host::control::SetupPacket::get_config_descriptor(0, 9);
    let mut buf = [0u8; 256];
    let len = bus.control_in(0, 0, mps, &get_config.to_bytes(), &mut buf)?;

    debug!(
        "raw configuration descriptor: {:x}",
        Debug2Format(&buf[..len])
    );

    match embassy_usb_host::descriptor::ConfigurationDescriptor::try_from_bytes(&buf[..len]) {
        Ok(desc) => {
            debug!("configuration descriptor: {:?}", Debug2Format(&desc));
            Ok(desc.total_len)
        }
        Err(e) => {
            error!(
                "failed to parse configuration descriptor {:x}: {}",
                Debug2Format(&buf[..len]),
                e
            );
            Err(PipeError::Stall)
        }
    }
}
fn get_all_device_configs<PIO: UsbPioInstance>(
    bus: &mut rp_pio_usb_host::bus::Bus<PIO>,
    mps: u16,
    max_len: u16,
) -> Result<u16, PipeError> {
    debug!(
        "getting configuration descriptor with max_len={:?}",
        max_len
    );
    let get_config = embassy_usb_host::control::SetupPacket::get_config_descriptor(0, max_len);
    let mut buf = [0u8; 256];
    let len = bus.control_in(0, 0, mps, &get_config.to_bytes(), &mut buf)?;

    debug!(
        "raw configuration descriptor: {:x}",
        Debug2Format(&buf[..len])
    );

    match embassy_usb_host::descriptor::ConfigurationDescriptorChain::try_from_slice(&buf[..len]) {
        Ok(chain) => {
            for desc in chain.iter_interface() {
                debug!("interface descriptor: {:?}", Debug2Format(&*desc));
            }
            Ok(chain.total_len)
        }
        Err(e) => {
            error!(
                "failed to parse configuration descriptor {:x}: {}",
                Debug2Format(&buf[..len]),
                e
            );
            Err(PipeError::Stall)
        }
    }
}

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

        let mps = match show_device_descriptor(&mut bus) {
            Ok(mps) => mps,
            Err(_) => continue,
        };

        let config_max_len = match get_device_config(&mut bus, mps) {
            Ok(len) => len,
            Err(_) => continue,
        };

        get_all_device_configs(&mut bus, mps, config_max_len).ok();
    }
}
