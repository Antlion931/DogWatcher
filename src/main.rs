#![allow(dead_code)]
#![no_std]
#![no_main]
use core::mem::discriminant;

use embedded_hal::digital::v2::InputPin;
use fugit::HertzU32;
use fugit::MicrosDurationU32;
use fugit::RateExtU32;
use hal::Clock;
use rp_pico as rpp;
/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
// The macro for marking our interrupt functions
use rp_pico::hal::pac::interrupt;
/// The USB Serial Device Driver (shared with the interrupt).
static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;
use hal::Spi;
// The macro for our start-up function
use rpp::entry;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rpp::hal::pac;
use rpp::hal::Timer;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rpp::hal;

// A few traits required for using the CountDown timer
use embedded_hal::timer::CountDown;
use fugit::ExtU32;
// USB Device support
//
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;
// Used to demonstrate writing formatted strings
use core::fmt::Write;
use heapless::String;
fn measure_distance(
    timer: &Timer,
    echo_pin: &dyn InputPin<Error = core::convert::Infallible>,
    trigger_pin: &mut dyn OutputPin<Error = core::convert::Infallible>,
) -> f64 {
    let mut delay = timer.count_down();
    trigger_pin.set_low().unwrap();
    delay.start(5.micros());
    let _ = nb::block!(delay.wait());

    // Set the trigger pin high and start the measurement timer
    trigger_pin.set_high().unwrap();
    delay.start(10.micros());
    let _ = nb::block!(delay.wait());

    // Measure the time until the echo pin becomes low again
    let start_time = timer.get_counter();
    while echo_pin.is_low().unwrap() {}
    let end_time = timer.get_counter();

    // Calculate the elapsed time in microseconds
    let elapsed_micros = end_time - start_time;

    // Calculate the distance based on the elapsed time and the speed of sound
    let distance_cm = elapsed_micros.to_micros() as f64 / 2.0 * 29.1;
    distance_cm
}

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let cp = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rpp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();


    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rpp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure the timer peripheral to be a CountDown timer for our blinky delay
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut delay = timer.count_down();

    // Set the LED to be an output
    let mut red_led = pins.gpio15.into_push_pull_output();
    let mut green_led = pins.gpio14.into_push_pull_output();
    let mut yellow_led = pins.gpio13.into_push_pull_output();
    // ultrasonic
    let mut trigger = pins.gpio17.into_push_pull_output();
    let echo = pins.gpio16.into_pull_up_input();
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }

    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB Communications Class Device driver
    let serial = SerialPort::new(bus_ref);
    unsafe {
        USB_SERIAL = Some(serial);
    }

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    // Enable the USB interrupt
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };
    loop {
        let distance = measure_distance(&timer, &echo, &mut trigger) as u32;
        red_led.set_high().unwrap();
        unsafe {
            let serial = USB_SERIAL.as_mut().unwrap();
            green_led.set_high().unwrap();
            let mut text: String<64> = String::new();
            writeln!(&mut text, "distance: {}\r\n", distance).unwrap();

            // This only works reliably because the number of bytes written to
            // the serial port is smaller than the buffers available to the USB
            // peripheral. In general, the return value should be handled, so that
            // bytes not transferred yet don't get lost.
            if serial.write(text.as_bytes()).is_err() {
                yellow_led.set_high().unwrap();
            }
        }

        delay.start(1000.millis());
        let _ = nb::block!(delay.wait());

        red_led.set_low().unwrap();
        green_led.set_low().unwrap();
        yellow_led.set_low().unwrap();
        delay.start(1000.millis());
        let _ = nb::block!(delay.wait());
    }
}
/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
///
/// We do all our USB work under interrupt, so the main thread can continue on
/// knowing nothing about USB.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    use core::sync::atomic::{AtomicBool, Ordering};

    /// Note whether we've already printed the "hello" message.
    static SAID_HELLO: AtomicBool = AtomicBool::new(false);

    // Grab the global objects. This is OK as we only access them under interrupt.
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let serial = USB_SERIAL.as_mut().unwrap();

    // Poll the USB driver with all of our supported USB Classes
    if usb_dev.poll(&mut [serial]) {
        
    }
}
