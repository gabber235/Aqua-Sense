#![no_std]
#![no_main]

extern crate alloc;

use alloc::{format, string::String};

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    i2c::I2C,
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use fugit::RateExtU32;

// use alloc::{format, string::String};

use lcd_1602_i2c::{Lcd, LcdDisplay};

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// Device I2c addresses
const LCD_ADDRESS: u8 = 0x7c >> 1;

#[entry]
fn main() -> ! {
    // Initialize the allocator BEFORE you use it
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let clocks = init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let sda_pin = pins.gpio4.into_function();
    let scl_pin = pins.gpio5.into_function();

    // Configure the I2C peripheral
    let i2c = I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let mut board_led = pins.led.into_push_pull_output();
    board_led.set_high().unwrap();

    let lcd = Lcd::new(i2c, LCD_ADDRESS, None, &mut delay);
    if let Err(_) = lcd {
        pins.gpio21.into_push_pull_output().set_high().unwrap();
    }
    let mut lcd = lcd.unwrap();

    let mut i = 0;

    loop {
        lcd.clear(&mut delay).unwrap();
        let text = format!("Tijd: {}", format_seconds(i));
        lcd.write_str(text.as_str()).unwrap();

        i += 1;

        delay.delay_ms(1000u32);
    }
}

fn format_seconds(seconds: u32) -> String {
    let hours = seconds / 3600;
    let minutes = (seconds - (hours * 3600)) / 60;
    let seconds = seconds - (hours * 3600) - (minutes * 60);

    if hours > 0 {
        return format!("{}h {}m {}s", hours, minutes, seconds);
    }

    if minutes > 0 {
        return format!("{}m {}s", minutes, seconds);
    }

    format!("{}s", seconds)
}

// End of file
