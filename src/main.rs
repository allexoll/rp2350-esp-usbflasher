//! # Pico USB Serial Example
//!
//! Creates a USB Serial device on a Pico board, with the USB driver running in
//! the main thread.
//!
//! This will create a USB Serial device echoing anything it receives. Incoming
//! ASCII characters are converted to upercase, so you can tell it is working
//! and not just local-echo!
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use embedded_hal::digital::{OutputPin, PinState, StatefulOutputPin};
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp235x_hal::{
    self as hal,
    uart::{DataBits, StopBits, UartConfig},
    Clock,
};

// Some things we need
use fugit::RateExtU32;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::{SerialPort, USB_CLASS_CDC};

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the rp235x peripherals, then writes to the UART in
/// an infinite loop.
#[hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USB,
        pac.USB_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led = pins.gpio7.into_push_pull_output_in_state(PinState::Low);
    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("ESP_PORT")])
        .unwrap()
        .device_class(USB_CLASS_CDC) // from: https://www.usb.org/defined-class-codes
        .build();

    led.set_high().unwrap();

    let uart_esp_pins = (
        // UART TX (characters sent from rp235x) on pin 4 (GPIO2) in Aux mode
        pins.gpio4.into_function(),
        // UART RX (characters received by rp235x) on pin 5 (GPIO3) in Aux mode
        pins.gpio5.into_function(),
    );
    let mut uart_esp = hal::uart::UartPeripheral::new(pac.UART1, uart_esp_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let mut esp32_boot_pin = pins.gpio14.into_push_pull_output_in_state(PinState::High);
    let mut esp32_reset_n = pins.gpio15.into_push_pull_output_in_state(PinState::High);
    let mut line: LineCoding = serial.line_coding().into();
    loop {
        // poll read
        let mut buf = [0u8; 8];
        match uart_esp.read_raw(&mut buf) {
            Err(_e) => {
                // Do nothing
            }
            Ok(0) => {
                // Do nothing
            }
            Ok(count) => {
                let _ = serial.write(&buf[..count]);
                let _ = serial.flush();
            }
        }
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }
        led.toggle().unwrap();
        match (serial.dtr(), serial.rts()) {
            (false, true) => {
                esp32_reset_n.set_low().unwrap();
                esp32_boot_pin.set_high().unwrap();
            }
            (true, false) => {
                esp32_reset_n.set_high().unwrap();
                esp32_boot_pin.set_low().unwrap();
            }
            _ => {
                esp32_reset_n.set_high().unwrap();
                esp32_boot_pin.set_high().unwrap();
            }
        }

        // // Check for new data
        let new_line: LineCoding = serial.line_coding().into();
        if new_line != line {
            line = new_line;
            let x = uart_esp.free();
            uart_esp = hal::uart::UartPeripheral::new(x.0, x.1, &mut pac.RESETS)
                .enable(
                    UartConfig::new(line.data_rate.Hz(), DataBits::Eight, None, StopBits::One),
                    clocks.peripheral_clock.freq(),
                )
                .unwrap();
        }
        let mut buf = [0u8; 64];
        match serial.read(&mut buf) {
            Err(_e) => {
                // Do nothing
            }
            Ok(0) => {
                // Do nothing
            }
            Ok(count) => {
                uart_esp.write_full_blocking(&buf[..count]);
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LineCoding {
    pub data_rate: u32,
}

impl From<&usbd_serial::LineCoding> for LineCoding {
    fn from(line_coding: &usbd_serial::LineCoding) -> Self {
        Self {
            data_rate: line_coding.data_rate(),
        }
    }
}
