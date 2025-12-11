#![no_std]
#![no_main]

// IMPORTS
use embassy_executor::Spawner;
use embassy_stm32::{gpio::{Level, Output, Speed}, peripherals::{PA9, PD5, PD12, PD13, PD14, PD15}, time::Hertz};
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::peripherals::PA0;

use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;

use embassy_stm32::spi::{Spi, Config as SpiConfig, Mode, Phase, Polarity};
use embassy_sync::channel::Channel;

use embassy_sync::signal::Signal;
use core::sync::atomic::{AtomicU8, Ordering};

use panic_probe as _;

// STATIC GLOBAL VARIABLES
static GREEN_LED2: Mutex<ThreadModeRawMutex, Option<Output<'static, PD12>>> = Mutex::new(None);
static ORANGE_LED: Mutex<ThreadModeRawMutex, Option<Output<'static, PD13>>> = Mutex::new(None);
static RED_LED2:   Mutex<ThreadModeRawMutex, Option<Output<'static, PD14>>> = Mutex::new(None);
static BLUE_LED:   Mutex<ThreadModeRawMutex, Option<Output<'static, PD15>>> = Mutex::new(None);
static GREEN_LED:  Mutex<ThreadModeRawMutex, Option<Output<'static, PA9>>>  = Mutex::new(None);
static RED_LED:    Mutex<ThreadModeRawMutex, Option<Output<'static, PD5>>>  = Mutex::new(None);

// Channel buffer depth = 4 messages
static ACCELEROMETER_CHANNEL: Channel<ThreadModeRawMutex, (i16, i16, i16), 4> = Channel::new();

static BUTTON_SIGNAL: Signal<ThreadModeRawMutex, ()> = Signal::new();
static MODE: AtomicU8 = AtomicU8::new(0); // 0 = blinky mode, 1 = accelerometer mode

// INITIALIZE CLOCK
fn clock_config() -> embassy_stm32::Config {
    let mut config = embassy_stm32::Config::default();
    config.rcc.hsi = true;
    config
}

// MAIN
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Call init ONCE
    let peripherals = embassy_stm32::init(clock_config());

    // Initialize the LEDs
    {
        let mut s = GREEN_LED2.lock().await;
        *s = Some(Output::new(peripherals.PD12, Level::High, Speed::VeryHigh));
    }
    {
        let mut s = ORANGE_LED.lock().await;
        *s = Some(Output::new(peripherals.PD13, Level::High, Speed::VeryHigh));
    }
    {
        let mut s = RED_LED2.lock().await;
        *s = Some(Output::new(peripherals.PD14, Level::High, Speed::VeryHigh));
    }
    {
        let mut s = BLUE_LED.lock().await;
        *s = Some(Output::new(peripherals.PD15, Level::High, Speed::VeryHigh));
    }
    {
        let mut s = GREEN_LED.lock().await;
        *s = Some(Output::new(peripherals.PA9, Level::High, Speed::VeryHigh));
    }
    {
        let mut s = RED_LED.lock().await;
        *s = Some(Output::new(peripherals.PD5, Level::High, Speed::VeryHigh));
    }

    // Initialize the Button
    let button = ExtiInput::new(Input::new(peripherals.PA0, Pull::Down), peripherals.EXTI0);

    // Initialize the Accelerometer
    let mut spi_cfg = SpiConfig::default();
    spi_cfg.mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    spi_cfg.frequency = Hertz(1_000_000);
    let spi = Spi::new(
        peripherals.SPI1,
        peripherals.PA5,
        peripherals.PA7,
        peripherals.PA6,
        peripherals.DMA2_CH3,
        peripherals.DMA2_CH2,
        spi_cfg,
    );
    let cs = Output::new(peripherals.PE3, Level::High, Speed::VeryHigh);

    // Spawn the Tasks
    spawner.spawn(green_blinky()).unwrap();
    spawner.spawn(orange_blinky()).unwrap();
    spawner.spawn(red_blinky()).unwrap();
    spawner.spawn(blue_blinky()).unwrap();
    spawner.spawn(button_irq(button)).unwrap();
    spawner.spawn(switch_application_mode()).unwrap();
    spawner.spawn(accelerometer_read(spi, cs)).unwrap();
    spawner.spawn(accelerometer_consumer()).unwrap();

    // Main Task Loop
    loop {
        embassy_time::Timer::after_millis(1000).await;
    }
}

// ----------------------------------------- TASKS ----------------------------------------- //

// Green LED Blinky Task
#[embassy_executor::task]
async fn green_blinky() {
    loop {
        if MODE.load(Ordering::Relaxed) == 0 {
            let mut s = GREEN_LED2.lock().await;
            if let Some(led) = s.as_mut() { led.toggle(); }
        }
        embassy_time::Timer::after_millis(200).await;
    }
}

// Orange LED Blinky Task
#[embassy_executor::task]
async fn orange_blinky() {
    loop {
        if MODE.load(Ordering::Relaxed) == 0 {
            let mut s = ORANGE_LED.lock().await;
            if let Some(led) = s.as_mut() { led.toggle(); }
        }
        embassy_time::Timer::after_millis(400).await;
    }
}

// Red LED Blinky Task
#[embassy_executor::task]
async fn red_blinky() {
    loop {
        if MODE.load(Ordering::Relaxed) == 0 {
            let mut s = RED_LED2.lock().await;
            if let Some(led) = s.as_mut() { led.toggle(); }
        }
        embassy_time::Timer::after_millis(800).await;
    }
}

// Blue LED Blinky Task
#[embassy_executor::task]
async fn blue_blinky() {
    loop {
        if MODE.load(Ordering::Relaxed) == 0 {
            let mut s = BLUE_LED.lock().await;
            if let Some(led) = s.as_mut() { led.toggle(); }
        }
        embassy_time::Timer::after_millis(1600).await;
    }
}

// Button Interrupt Task
// An interrupt which runs when the User Button is pressed, it signals switch_application_mode
#[embassy_executor::task]
async fn button_irq(mut button: ExtiInput<'static, PA0>) {
    loop {
        button.wait_for_rising_edge().await;

        BUTTON_SIGNAL.signal(());

        embassy_time::Timer::after_millis(100).await;
    }
}

// Switch Application Mode Task
// Waits for the signal from the button interrupt task. Once recieved, it changes the application mode and alternates between lighting 2 LEDs
// Modes:
// Mode 1: Blinky Mode
// Mode 2: Accelerometer Mode
#[embassy_executor::task]
async fn switch_application_mode() {
    loop {
        BUTTON_SIGNAL.wait().await;

        {
            let mut s = GREEN_LED.lock().await;
            if let Some(led) = s.as_mut() { led.toggle(); }
        }
        {
            let mut s = RED_LED.lock().await;
            if let Some(led) = s.as_mut() { led.toggle(); }
        }

        let current = MODE.load(Ordering::Relaxed);
        let new = if current == 0 { 1 } else { 0 };
        MODE.store(new, Ordering::Relaxed);
    }
}

// Read Accelerometer Task
// Reads the accelerometer value and sends it to accelerometer_consumer
#[embassy_executor::task]
async fn accelerometer_read(
    mut spi: Spi<'static, embassy_stm32::peripherals::SPI1,
                 embassy_stm32::peripherals::DMA2_CH3,
                 embassy_stm32::peripherals::DMA2_CH2>,
    mut cs: Output<'static, embassy_stm32::peripherals::PE3>,
) {
    cs.set_low();
    spi.write(&[0x20u8, 0x67u8]).await.unwrap();
    cs.set_high();

    loop {
        if MODE.load(Ordering::Relaxed) == 1 {
            let mut buf = [0u8; 7];
            buf[0] = 0x28 | 0x80;
            cs.set_low();
            spi.transfer_in_place(&mut buf).await.unwrap();
            cs.set_high();

            let x = i16::from_le_bytes([buf[1], buf[2]]);
            let y = i16::from_le_bytes([buf[3], buf[4]]);
            let z = i16::from_le_bytes([buf[5], buf[6]]);


            ACCELEROMETER_CHANNEL.send((x, y, z)).await;
        }

        embassy_time::Timer::after_millis(100).await;
    }
}

// Consume Accelerometer Data Task
// Receives the value read from the accelerometer_read and lights the respective LED
#[embassy_executor::task]
async fn accelerometer_consumer() {

    // Thresholds in milli-g (adjustable)
    let threshold: i16 = 2000;

    loop {
        if MODE.load(Ordering::Relaxed) == 1 {
            let (x, y, _z) = ACCELEROMETER_CHANNEL.receive().await;

            // Clear all LEDs first
            {
                let mut s = GREEN_LED2.lock().await;
                if let Some(led) = s.as_mut() { led.set_low(); }
            }
            {
                let mut s = ORANGE_LED.lock().await;
                if let Some(led) = s.as_mut() { led.set_low(); }
            }
            {
                let mut s = RED_LED2.lock().await;
                if let Some(led) = s.as_mut() { led.set_low(); }
            }
            {
                let mut s = BLUE_LED.lock().await;
                if let Some(led) = s.as_mut() { led.set_low(); }
            }

            if x < -threshold {
                let mut s = GREEN_LED2.lock().await;
                if let Some(led) = s.as_mut() { led.set_high(); }
            } 
            if y > threshold {
                let mut s = ORANGE_LED.lock().await;
                if let Some(led) = s.as_mut() { led.set_high(); }
            } 
            if x > threshold {
                let mut s = RED_LED2.lock().await;
                if let Some(led) = s.as_mut() { led.set_high(); }
            } 
            if y < -threshold {
                let mut s = BLUE_LED.lock().await;
                if let Some(led) = s.as_mut() { led.set_high(); }
            }
        }

        embassy_time::Timer::after_millis(50).await;
    }
}
