#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::cell::RefCell;
use critical_section::Mutex;
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use {esp_backtrace as _, esp_println as _};
use esp_hal::gpio::{Output, OutputConfig, Level, Input, InputConfig, Event, Io};
use esp_hal::{handler, ram};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

static BOOT_BUTTON: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.5.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    let rng = esp_hal::rng::Rng::new(peripherals.RNG);
    let timer1 = TimerGroup::new(peripherals.TIMG0);
    let wifi_init =
        esp_wifi::init(timer1.timer0, rng).expect("Failed to initialize WIFI/BLE controller");
    let (mut _wifi_controller, _interfaces) = esp_wifi::wifi::new(&wifi_init, peripherals.WIFI)
        .expect("Failed to initialize WIFI controller");

    let mut io = Io::new(peripherals.IO_MUX);
    io.set_interrupt_handler(handler);

    // 分配 GPIO 引脚
    let led = Output::new(peripherals.GPIO1, Level::Low, OutputConfig::default());
    let mut boot_button = Input::new(peripherals.GPIO0, InputConfig::default());

    critical_section::with(|cs| {
        boot_button.listen(Event::FallingEdge);
        BOOT_BUTTON.borrow_ref_mut(cs).replace(boot_button)
    });

    spawner.spawn(run()).expect("run spawn failed");
}

#[embassy_executor::task]
async fn run() {
    loop {
        info!("tick");
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn button_task(boot_button: Input<'static>, mut led: Output<'static>) {
    // 初始化 LED 状态 (假设初始为关闭)
    led.set_low();
    let mut last_button_state = false;

    loop {
        let current_button_state = boot_button.is_high();
        if current_button_state && !last_button_state {
            Timer::after(Duration::from_millis(20)).await;
            if boot_button.is_high() {
                info!("Button pressed!");
                led.toggle();
            }
        }
        last_button_state = current_button_state;
        Timer::after(Duration::from_millis(10)).await;
    }
}

#[handler]
#[ram]
fn handler() {
    info!(
        "GPIO Interrupt with priority {}",
        esp_hal::xtensa_lx::interrupt::get_level()
    );

    if critical_section::with(|cs| {
        BOOT_BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .is_interrupt_set()
    }) {
        info!("boot button was the source of the interrupt");
    } else {
        warn!("Button was not the source of the interrupt");
    }

    critical_section::with(|cs| {
        BOOT_BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}