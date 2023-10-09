#![no_std]
#![no_main]
extern crate embedded_hal;
extern crate fugit;
extern crate lcd_1602_i2c;
extern crate panic_halt;
extern crate rp2040_hal;
#[macro_use]
extern crate alloc;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use fugit::RateExtU32;
use hal::{gpio::Pins, pac, Sio};

// Some traits we need
use lcd_1602_i2c::{Cursor, Lcd};
use rp2040_hal::clocks::Clock;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// Device I2c addresses
const LCD_ADDRESS: u8 = 0x7c >> 1;
const RGB_ADDRESS: u8 = 0xc0 >> 1;

#[rp2040_hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut peripherals = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(peripherals.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        peripherals.XOSC,
        peripherals.CLOCKS,
        peripherals.PLL_SYS,
        peripherals.PLL_USB,
        &mut peripherals.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = Sio::new(peripherals.SIO);

    // Set the pins to their default state
    let pins = Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );

    // Configure the I2C peripheral
    let mut i2c = hal::I2C::i2c0(
        peripherals.I2C0,
        pins.gpio0.into_function(),
        pins.gpio1.into_function(),
        400.kHz(),
        &mut peripherals.RESETS,
        &clocks.system_clock,
    );

    let mut lcd = Lcd::new(i2c, LCD_ADDRESS, RGB_ADDRESS, &mut delay).unwrap();
    lcd.set_rgb(255, 255, 255).unwrap();
    lcd.set_cursor(Cursor::On).unwrap();

    let mut i = 0;

    loop {
        lcd.clear(&mut delay).unwrap();
        lcd.write_str(format!("S: {}", i).as_str()).unwrap();
        i += 1;
        delay.delay_ms(1000u32);
    }
}

// End of file
