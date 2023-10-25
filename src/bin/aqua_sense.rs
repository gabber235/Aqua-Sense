#![no_std]
#![no_main]

extern crate alloc;

use core::cell::RefCell;

use alloc::{format, string::String};

use bsp::{
    entry,
    hal::{
        gpio::{
            self,
            bank0::{Gpio4, Gpio5},
            FunctionI2c, Pin, PullDown,
        },
        pwm::{self, InputHighRunning},
        timer::{Alarm, Alarm0},
        Timer,
    },
    pac::{interrupt, I2C0},
};

use cortex_m::interrupt::Mutex;
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
use fugit::{ExtU32, RateExtU32};

// use alloc::{format, string::String};

use lcd_1602_i2c::Lcd;

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// Device I2c addresses
const LCD_ADDRESS: u8 = 0x7c >> 1;

const PULSES_PER_LITER: f32 = 341.0;

const SCREEN_WIDTH: u8 = 16;

// Color cycle
const COLOR_CYCLE: [Rgb; 6] = [
    Rgb(false, true, false), // Green
    Rgb(false, false, true), // Blue
    Rgb(false, true, true),  // Cyan
    Rgb(true, true, false),  // Yellow
    Rgb(true, false, true),  // Magenta
    Rgb(true, false, false), // Red
];

/// This pin will be our input for a 50 Hz servo PWM signal
type InputPwmPin = gpio::Pin<gpio::bank0::Gpio9, gpio::FunctionPwm, gpio::PullDown>;

/// This will be our PWM Slice - it will interpret the PWM signal from the pin
type PwmSlice = pwm::Slice<pwm::Pwm4, pwm::InputHighRunning>;

/// Since we're always accessing these pins together we'll store them in a tuple.
/// Giving this tuple a type alias means we won't need to use () when putting them
/// inside an Option. That will be easier to read.
type LedInputAndPwm = (InputPwmPin, PwmSlice);

type Display = Lcd<
    I2C<
        I2C0,
        (
            Pin<Gpio4, FunctionI2c, PullDown>,
            Pin<Gpio5, FunctionI2c, PullDown>,
        ),
    >,
>;

type RgbLed = (
    Pin<gpio::bank0::Gpio18, gpio::FunctionSio<gpio::SioOutput>, PullDown>,
    Pin<gpio::bank0::Gpio17, gpio::FunctionSio<gpio::SioOutput>, PullDown>,
    Pin<gpio::bank0::Gpio16, gpio::FunctionSio<gpio::SioOutput>, PullDown>,
);

static FLOW_PINS: Mutex<RefCell<Option<LedInputAndPwm>>> = Mutex::new(RefCell::new(None));
static DISPLAY: Mutex<RefCell<Option<Display>>> = Mutex::new(RefCell::new(None));
static RGB_LED: Mutex<RefCell<Option<RgbLed>>> = Mutex::new(RefCell::new(None));
static DELAY: Mutex<RefCell<Option<cortex_m::delay::Delay>>> = Mutex::new(RefCell::new(None));
static ALARM: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));

static COUNTER: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));
static STATE: Mutex<RefCell<State>> = Mutex::new(RefCell::new(State::Off));

#[entry]
fn main() -> ! {
    // ------------------ Initialize the heap allocator ------------------
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }
    // -------------------------------------------------------------------

    // ------------------------ Initialize device ------------------------
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
    // -------------------------------------------------------------------

    // ------------------------ Initialize peripherals ------------------------
    let rgb_led: RgbLed = (
        pins.gpio18.into_push_pull_output(),
        pins.gpio17.into_push_pull_output(),
        pins.gpio16.into_push_pull_output(),
    );

    // ----- Configure display -----
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

    let lcd = Lcd::new(i2c, LCD_ADDRESS, None, &mut delay);
    if let Err(_) = lcd {
        pins.gpio21.into_push_pull_output().set_high().unwrap();
    }
    let lcd = lcd.unwrap();
    // -----------------------------

    // ----- Configure flow sensor -----
    let pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM0 slice
    // The PWM slice clock should only run when the input is high (InputHighRunning)
    let mut pwm = pwm_slices.pwm4.into_mode::<InputHighRunning>();

    // Divide the 125 MHz system clock by 125 to give a 1 MHz PWM slice clock (1 us per tick)
    pwm.set_div_int(125); //TODO: check if this is correct
    pwm.enable();

    // Connect to GPI O1 as the input to channel B on PWM0
    let channel = &mut pwm.channel_b;
    let input_pin = channel.input_from(pins.gpio9);

    // Enable an interrupt whenever GPI O1 goes from high to low (the end of a pulse)
    input_pin.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);
    // -----------------------------

    // ----- Configure timer -----
    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut alarm = timer.alarm_0().unwrap();

    alarm.schedule(1.secs().into()).unwrap();
    alarm.enable_interrupt();

    cortex_m::interrupt::free(|cs| {
        RGB_LED.borrow(cs).replace(Some(rgb_led));
        DISPLAY.borrow(cs).replace(Some(lcd));
        FLOW_PINS.borrow(cs).replace(Some((input_pin, pwm)));
        DELAY.borrow(cs).replace(Some(delay));
        ALARM.borrow(cs).replace(Some(alarm));
    });

    #[allow(unsafe_code)]
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0); // Adjust interrupt name accordingly
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
        cortex_m::interrupt::enable();
    }

    loop {
        cortex_m::asm::wfi();
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    // The `#[interrupt]` attribute covertly converts this to `&'static mut Option<LedAndInput>`
    static mut LED_INPUT_AND_PWM: Option<LedInputAndPwm> = None;

    // This is one-time lazy initialisation. We steal the variables given to us
    // via `GLOBAL_PINS`.
    if LED_INPUT_AND_PWM.is_none() {
        cortex_m::interrupt::free(|cs| {
            *LED_INPUT_AND_PWM = FLOW_PINS.borrow(cs).take();
        });
    }

    if let Some((input, _)) = LED_INPUT_AND_PWM {
        if !input.interrupt_status(gpio::Interrupt::EdgeLow) {
            return;
        }

        // Our interrupt doesn't clear itself.
        // Do that now so we don't immediately jump back to this interrupt handler.
        input.clear_interrupt(gpio::Interrupt::EdgeLow);

        on_pulse();
    }
}

fn on_pulse() {
    cortex_m::interrupt::free(|cs| {
        let mut counter_ref = COUNTER.borrow(cs).borrow_mut();

        *counter_ref += 1;
    });
}

#[interrupt]
fn TIMER_IRQ_0() {
    static mut TIMER: Option<Alarm0> = None;

    if TIMER.is_none() {
        cortex_m::interrupt::free(|cs| {
            *TIMER = ALARM.borrow(cs).take();
        });
    }

    if let Some(alarm) = TIMER {
        alarm.clear_interrupt();
        alarm.schedule(1.secs().into()).unwrap();
        alarm.enable_interrupt();

        on_timer();
    }
}

fn on_timer() {
    let counter = read_and_reset_counter();
    let state = cortex_m::interrupt::free(|cs| {
        let mut state_ref = STATE.borrow(cs).borrow_mut();

        let new_state = state_ref.tick(counter);

        *state_ref = new_state.clone();
        new_state
    });

    state.display();
}

fn write_lcd(line1: &str, line2: &str) {
    cortex_m::interrupt::free(|cs| {
        if let Some(lcd) = DISPLAY.borrow(cs).borrow_mut().as_mut() {
            if let Some(delay) = DELAY.borrow(cs).borrow_mut().as_mut() {
                lcd.clear(delay).unwrap();
            }
            lcd.write_str(line1).unwrap();
            lcd.set_cursor_position(0, 1).unwrap();
            lcd.write_str(line2).unwrap();
        }
    });
}

fn set_rgb(rgb: Rgb) {
    cortex_m::interrupt::free(|cs| {
        if let Some((r, g, b)) = RGB_LED.borrow(cs).borrow_mut().as_mut() {
            r.set_state(rgb.red()).unwrap();
            g.set_state(rgb.green()).unwrap();
            b.set_state(rgb.blue()).unwrap();
        }
    });
}

fn read_and_reset_counter() -> u32 {
    cortex_m::interrupt::free(|cs| {
        let mut counter_ref = COUNTER.borrow(cs).borrow_mut();

        let counter = *counter_ref;
        *counter_ref = 0;

        counter
    })
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

fn format_liters(liters: f32) -> String {
    format!("{:.2}", liters)
}

fn multi_part_lines(left: String, right: String) -> String {
    if left.len() + right.len() <= SCREEN_WIDTH {
        format! {
            "{}{}{}",
            left,
            " ".repeat(SCREEN_WIDTH - left.len() - right.len()),
            right
        }
    } else {
        left
    }
}

fn center_text(text: String) -> String {
    if text.len() < SCREEN_WIDTH {
        format!("{}{}", " ".repeat((SCREEN_WIDTH - text.len()) / 2), text)
    } else {
        text
    }
}

#[derive(Clone, Copy)]
struct Rgb(bool, bool, bool);

impl Rgb {
    fn off() -> Self {
        Rgb(false, false, false)
    }
}

macro_rules! rgb_to_pin_state {
    ($name:ident, $index:tt) => {
        impl Rgb {
            fn $name(&self) -> gpio::PinState {
                if self.$index {
                    gpio::PinState::High
                } else {
                    gpio::PinState::Low
                }
            }
        }
    };
}

rgb_to_pin_state!(red, 0);
rgb_to_pin_state!(green, 1);
rgb_to_pin_state!(blue, 2);

#[derive(Clone)]
enum State {
    Off,
    Running {
        liters: f32,
        flow_rate: f32,
        seconds: u32,
    },
    Stopped {
        liters: f32,
        seconds: u32,
        countdown: u32,
    },
}

impl State {
    fn tick(&self, count: u32) -> Self {
        let additional_liters = (count as f32) / PULSES_PER_LITER;
        let flow_rate = additional_liters * 60.0;
        match self {
            State::Off => {
                if count > 0 {
                    State::Running {
                        liters: additional_liters,
                        flow_rate,
                        seconds: 0,
                    }
                } else {
                    State::Off
                }
            }
            State::Running {
                liters, seconds, ..
            } => {
                if count > 0 {
                    State::Running {
                        liters: liters + additional_liters,
                        flow_rate,
                        seconds: seconds + 1,
                    }
                } else {
                    State::Stopped {
                        liters: *liters,
                        seconds: *seconds,
                        countdown: 5,
                    }
                }
            }
            State::Stopped {
                liters,
                seconds,
                countdown,
            } => {
                if count > 0 {
                    State::Running {
                        liters: liters + additional_liters,
                        flow_rate,
                        seconds: seconds + 1,
                    }
                } else if *countdown > 0 {
                    State::Stopped {
                        liters: *liters,
                        seconds: *seconds,
                        countdown: *countdown - 1,
                    }
                } else {
                    State::Off
                }
            }
        }
    }

    fn curent_rgb(seconds: u32) -> Rgb {
        let index = (seconds / 10) % 6;
        COLOR_CYCLE[index as usize]
    }

    fn display(&self) {
        match self {
            State::Off => {
                write_lcd("", "");
                set_rgb(Rgb::off());
            }
            State::Running {
                liters,
                flow_rate,
                seconds,
            } => {
                let liters_text = format!("{}L", format_liters(*liters));
                let flow_text = format!("{:.2}L/m", flow_rate);
                let seconds_text = format!("{}", format_seconds(*seconds));

                let line1 = multi_part_lines(liters_text, flow_text);
                let line2 = center_text(seconds_text);

                write_lcd(line1.as_str(), line2.as_str());
                set_rgb(Self::curent_rgb(*seconds));
            }
            State::Stopped {
                liters,
                seconds,
                countdown,
            } => {
                let liters_text = format!("Used: {}L", format_liters(*liters));
                let countdown_text = format!("({})", countdown);
                let seconds_text = format!("{}", format_seconds(*seconds));

                let line1 = multi_part_lines(liters_text, countdown_text);
                let line2 = center_text(seconds_text);

                write_lcd(line1.as_str(), line2.as_str());

                if *countdown % 2 == 0 {
                    set_rgb(Self::curent_rgb(*seconds));
                } else {
                    set_rgb(Rgb::off());
                }
            }
        }
    }
}
