// Perform initial handshake with a USB device and light up the LED when connected.

#![no_std]
#![no_main]

use defmt::{Debug2Format, debug, error, info};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::InterruptHandler;
use embassy_usb_driver::host::{
    DeviceEvent, HostError, PipeError, TimeoutConfig, UsbHostAllocator, UsbHostController,
};
use embassy_usb_driver::host::{UsbPipe, pipe};
use embassy_usb_driver::{EndpointAddress, EndpointInfo, EndpointType};
use embassy_usb_host::descriptor::{
    ConfigurationDescriptor, DeviceDescriptor, DeviceDescriptorPartial, USBDescriptor,
};
use rp_pio_usb_host::Bus;
use rp_pio_usb_host::Pulldown;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[derive(defmt::Format)]
enum DescriptorError {
    Pipe(PipeError),
    Decode,
    Host(HostError),
}

impl From<PipeError> for DescriptorError {
    fn from(err: PipeError) -> Self {
        DescriptorError::Pipe(err)
    }
}

impl From<HostError> for DescriptorError {
    fn from(err: HostError) -> Self {
        DescriptorError::Host(err)
    }
}

fn control_pipe<'a>(
    controller: &impl UsbHostController<'a>,
    mps: u16,
) -> impl UsbPipe<pipe::Control, pipe::In> {
    let ep = EndpointInfo {
        addr: EndpointAddress::from_parts(0, embassy_usb_driver::Direction::In),
        ep_type: EndpointType::Control,
        max_packet_size: mps,
        interval_ms: 0,
    };
    controller.allocator().alloc_pipe(0, &ep, None).unwrap()
}

async fn get_partial_device_descriptor(
    controller: &impl UsbHostController<'_>,
) -> Result<DeviceDescriptorPartial, DescriptorError> {
    let get_device = embassy_usb_host::control::SetupPacket::get_device_descriptor(8);
    let mut pdd = [0u8; 64];

    let mut pipe = control_pipe(controller, 8);
    pipe.set_timeout(TimeoutConfig::default());
    let len = pipe.control_in(&get_device.to_bytes(), &mut pdd).await?;

    Ok(
        DeviceDescriptorPartial::try_from_bytes(&pdd[..len])
            .map_err(|_| DescriptorError::Decode)?,
    )
}

async fn get_full_device_descriptor(
    controller: &impl UsbHostController<'_>,
    mps: u16,
) -> Result<DeviceDescriptor, DescriptorError> {
    let get_device = embassy_usb_host::control::SetupPacket::get_device_descriptor(mps);
    let mut pdd = [0u8; 64];

    let mut pipe = control_pipe(controller, mps);
    pipe.set_timeout(TimeoutConfig::default());
    let len = pipe.control_in(&get_device.to_bytes(), &mut pdd).await?;

    Ok(DeviceDescriptor::try_from_bytes(&pdd[..len]).map_err(|_| DescriptorError::Decode)?)
}

async fn show_device_descriptor(
    controller: &impl UsbHostController<'_>,
) -> Result<u16, DescriptorError> {
    let partial_desc = get_partial_device_descriptor(controller).await?;
    debug!(
        "partial device descriptor: {:?}",
        Debug2Format(&partial_desc)
    );
    let mps = partial_desc.max_packet_size0 as u16;
    let desc = get_full_device_descriptor(controller, mps).await?;

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
    Ok(desc.max_packet_size0 as u16)
}

async fn get_device_config_len(
    controller: &impl UsbHostController<'_>,
    mps: u16,
) -> Result<u16, DescriptorError> {
    debug!("getting initial configuration descriptor",);
    let get_config = embassy_usb_host::control::SetupPacket::get_config_descriptor(0, 9);
    let mut buf = [0u8; 256];
    let mut pipe = control_pipe(controller, mps);

    let len = pipe.control_in(&get_config.to_bytes(), &mut buf).await?;
    debug!(
        "raw configuration descriptor: {:x}",
        Debug2Format(&buf[..len])
    );

    Ok(ConfigurationDescriptor::try_from_bytes(&buf[..len])
        .map_err(|_| DescriptorError::Decode)?
        .total_len)
}

async fn show_all_device_configs(
    controller: &impl UsbHostController<'_>,
    mps: u16,
    max_len: u16,
) -> Result<(), DescriptorError> {
    debug!(
        "getting configuration descriptor with max_len={:?}",
        max_len
    );
    let get_config = embassy_usb_host::control::SetupPacket::get_config_descriptor(0, max_len);
    let mut buf = [0u8; 256];
    let mut pipe = control_pipe(controller, mps);

    let len = pipe.control_in(&get_config.to_bytes(), &mut buf).await?;
    debug!(
        "raw configuration descriptor: {:x}",
        Debug2Format(&buf[..len])
    );

    let desc = ConfigurationDescriptor::try_from_slice(&buf[..len])
        .map_err(|_| DescriptorError::Decode)?;
    for iface in desc.iter_interface() {
        debug!("interface descriptor: {:?}", Debug2Format(&iface));
    }

    Ok(())
}

#[embassy_executor::task]
async fn usb_idle_task(bus: &'static Bus<'static, PIO0>) {
    bus.idle_task().await;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut led = Output::new(p.PIN_25, Level::Low);

    info!("waiting for device");
    static BUS: StaticCell<Bus<PIO0>> = StaticCell::new();
    let bus = BUS.init(Bus::new(p.PIO0, p.PIN_0, p.PIN_1, Irqs, Pulldown::External));
    spawner.spawn(usb_idle_task(bus).unwrap());

    let mut controller = bus.controller();
    loop {
        let event = controller.wait_for_device_event().await;
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

        let mps = match show_device_descriptor(&controller).await {
            Ok(mps) => mps,
            Err(e) => {
                error!("error showing device descriptor: {:?}", e);
                continue;
            }
        };

        let config_max_len = match get_device_config_len(&controller, mps).await {
            Ok(len) => len,
            Err(e) => {
                error!("error getting device config length: {:?}", e);
                continue;
            }
        };

        match show_all_device_configs(&controller, mps, config_max_len).await {
            Ok(_) => {}
            Err(e) => {
                error!("error showing all device configs: {:?}", e);
                continue;
            }
        }
    }
}
