#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use crate::eeprom::At24C02;
use core::cell::RefCell;
use critical_section::Mutex;
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Event, Input, InputConfig, Io, Level, Output, OutputConfig};
use esp_hal::i2c::master::Config as I2cConfig;
use esp_hal::i2c::master::I2c;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{handler, ram, Blocking};
use esp_wifi::wifi::WifiController;
use esp_wifi::EspWifiController;
use static_cell::StaticCell;
use {esp_backtrace as _, esp_println as _};

mod eeprom;
mod xl9555;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

static BOOT_BUTTON: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));
static LED: Mutex<RefCell<Option<Output>>> = Mutex::new(RefCell::new(None));
static I2C: Mutex<RefCell<Option<I2c<Blocking>>>> = Mutex::new(RefCell::new(None));
// 在全局静态变量中添加按键状态跟踪
// [KEY0, KEY1, KEY2, KEY3]
static KEY_STATES: Mutex<RefCell<[bool; 4]>> = Mutex::new(RefCell::new([false; 4]));

static WIFI_INIT: StaticCell<EspWifiController> = StaticCell::new();
static WIFI_CONTROLLER: Mutex<RefCell<Option<WifiController<'static>>>> =
    Mutex::new(RefCell::new(None));

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.5.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .expect("Failed to initialize I2C")
        .with_sda(peripherals.GPIO41)
        .with_scl(peripherals.GPIO42);

    critical_section::with(|cs| I2C.borrow_ref_mut(cs).replace(i2c));

    let rng = esp_hal::rng::Rng::new(peripherals.RNG);
    let timer1 = TimerGroup::new(peripherals.TIMG0);
    let wifi_init =
        esp_wifi::init(timer1.timer0, rng).expect("Failed to initialize WIFI/BLE controller");

    let wifi_init_static = WIFI_INIT.init(wifi_init);

    let (mut wifi_controller, _interfaces) =
        esp_wifi::wifi::new(wifi_init_static, peripherals.WIFI)
            .expect("Failed to initialize WIFI controller");

    // 设置Wi-Fi模式，例如STA模式
    wifi_controller
        .set_configuration(&esp_wifi::wifi::Configuration::Client(
            esp_wifi::wifi::ClientConfiguration::default(),
        ))
        .expect("Failed to set Wi-Fi configuration");

    match wifi_controller.start_async().await {
        Ok(()) => {
            info!("Wi-Fi started");
        }
        Err(err) => {
            warn!("Wi-Fi start failed: {}", err);
        }
    }

    match wifi_controller.is_started() {
        Ok(started) => {
            if started {
                info!("Wi-Fi started");
            } else {
                warn!("Wi-Fi not started");
            }
        }
        Err(err) => {
            warn!("Failed to check Wi-Fi started: {}", err);
        }
    };

    let result = wifi_controller.scan_n_async(10).await;

    match result {
        Ok(networks) => {
            info!("Scan done, found {} networks", networks.len());
            for network in networks {
                info!(
                    "SSID: {}, Channel: {}, RSSI: {}",
                    core::str::from_utf8((&network.ssid).as_ref()).unwrap_or("<invalid utf-8>"),
                    network.channel,
                    network.auth_method
                );
            }
        }
        Err(err) => {
            warn!("Wi-Fi scan failed: {}", err);
        }
    }

    critical_section::with(|cs| WIFI_CONTROLLER.borrow_ref_mut(cs).replace(wifi_controller));

    // setup interrupt
    let mut io = Io::new(peripherals.IO_MUX);
    io.set_interrupt_handler(interrupt_handler);

    // 分配 GPIO 引脚
    let led = Output::new(peripherals.GPIO1, Level::Low, OutputConfig::default());
    let mut boot_button = Input::new(peripherals.GPIO0, InputConfig::default());

    critical_section::with(|cs| {
        boot_button.listen(Event::FallingEdge);
        BOOT_BUTTON.borrow_ref_mut(cs).replace(boot_button)
    });

    critical_section::with(|cs| LED.borrow_ref_mut(cs).replace(led));

    // spawner.spawn(wifi_scan()).ok();
    spawner.spawn(i2c_scan()).ok();
    spawner.spawn(eeprom_demo()).ok();
    spawner.spawn(read_keys()).ok();
    // spawner.spawn(run()).expect("run spawn failed");
}

#[embassy_executor::task]
async fn run() {
    loop {
        info!("tick");
        Timer::after_secs(1).await;
    }
}

// #[embassy_executor::task]
// async fn wifi_scan() {
//     info!("Wifi Scanning...");
//
//     // 执行 Wi-Fi 扫描
//     critical_section::with(|cs| {
//         let mut wifi_controller = WIFI_CONTROLLER.borrow_ref_mut(cs);
//         let wifi_controller_ref = wifi_controller.as_mut().unwrap();
//
//         // 使用异步扫描，最多扫描5个网络
//         let scan_result = wifi_controller_ref.scan_n(5);
//
//         match scan_result {
//             Ok(networks) => {
//                 info!("Scan done, found {} networks", networks.len());
//                 networks.iter().for_each(|network| {
//                     info!(
//                         "SSID: {}, Channel: {}, RSSI: {}",
//                         core::str::from_utf8((&network.ssid).as_ref()).unwrap_or("<invalid utf-8>"),
//                         network.channel,
//                         network.auth_method
//                     );
//                 });
//             }
//             Err(err) => warn!("Wifi scan failed: {}", err),
//         };
//     });
// }

#[embassy_executor::task]
async fn i2c_scan() {
    info!("Starting I2C scan...");
    critical_section::with(|cs| {
        let mut i2c = I2C.borrow_ref_mut(cs);
        let i2c_ref = i2c.as_mut().unwrap();

        for addr in 0x08..=0x77 {
            match i2c_ref.write(addr, &[]) {
                Ok(_) => {
                    info!("Found device at address: 0x{:02X}", addr);
                }
                Err(_) => {}
            }
        }
    });
}

/**
* 读取按键输入
* 状态跟踪: 添加 KEY_STATES 全局变量记录每个按键的上一次状态
* 边缘检测: 只有当按键从释放状态(高电平)变为按下状态(低电平)时才触发事件
* 状态更新: 每次循环结束后更新按键状态数组
* 这样修改后，即使按键持续按下也只会触发一次日志输出，直到按键释放后再次按下才会重新触发
* 硬件连接：
* iic_int (XL9555中断引脚) 连接到 ESP32 的 GPIO0
* GPIO0 同时也是 BOOT_BUTTON 的引脚
* 中断触发机制：
* 当 KEY0-KEY3 按下时，XL9555 通过 iic_int 引脚产生中断信号
* 该信号传递到 GPIO0，触发了已注册的中断处理程序
* 中断处理程序中会切换 LED 状态
*/
#[embassy_executor::task]
async fn read_keys() {
    loop {
        critical_section::with(|cs| {
            let mut i2c = I2C.borrow_ref_mut(cs);
            let i2c_ref = i2c.as_mut().unwrap();

            // 读取端口0和端口1的输入值
            let mut port0_data = [0u8];
            let mut port1_data = [0u8];

            i2c_ref
                .write_read(
                    xl9555::XL9555_ADDR,
                    &[xl9555::regsisters::INPUT_PORT_0],
                    &mut port0_data,
                )
                .ok();
            i2c_ref
                .write_read(
                    xl9555::XL9555_ADDR,
                    &[xl9555::regsisters::INPUT_PORT_1],
                    &mut port1_data,
                )
                .ok();

            let key_value: u16 = (port1_data[0] as u16) << 8 | (port0_data[0] as u16);

            // 获取当前按键状态（低电平表示按下）
            let current_states = [
                (key_value & xl9555::io_bits::KEY0_IO) == 0,
                (key_value & xl9555::io_bits::KEY1_IO) == 0,
                (key_value & xl9555::io_bits::KEY2_IO) == 0,
                (key_value & xl9555::io_bits::KEY3_IO) == 0,
            ];

            // 检查按键状态变化
            let mut key_states = KEY_STATES.borrow_ref_mut(cs);
            for i in 0..4 {
                if current_states[i] && !key_states[i] {
                    // 按键刚被按下
                    match i {
                        0 => info!("KEY0 pressed"),
                        1 => info!("KEY1 pressed"),
                        2 => info!("KEY2 pressed"),
                        3 => info!("KEY3 pressed"),
                        _ => {}
                    }
                }
            }

            // 更新按键状态
            *key_states = current_states;
        });

        Timer::after_millis(50).await;
    }
}

/**
 * 响应中断
 */
#[handler]
#[ram]
fn interrupt_handler() {
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
        critical_section::with(|cs| {
            LED.borrow_ref_mut(cs).as_mut().unwrap().toggle();
        });
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

#[embassy_executor::task]
async fn eeprom_demo() {
    // 创建 EEPROM 实例 (假设设备地址为 0x50)
    let mut eeprom = At24C02::new(&I2C, 0x50);

    info!("EEPROM demo started");

    // 写入单个字节
    match eeprom.write_byte(0x00, 0xAB).await {
        Ok(_) => info!("Successfully wrote 0xAB to address 0x00"),
        Err(e) => warn!("Failed to write byte: {:?}", e),
    }

    // 等待一段时间让写操作完成（EEPROM写入需要时间）
    Timer::after_millis(10).await;

    // 读取刚才写入的字节
    match eeprom.read_byte(0x00).await {
        Ok(value) => info!("Read value from address 0x00: 0x{:02X}", value),
        Err(e) => warn!("Failed to read byte: {:?}", e),
    }

    // 写入一页数据
    let page_data = [0x11, 0x22, 0x33, 0x44];
    match eeprom.write_page(0x10, &page_data).await {
        Ok(_) => info!("Successfully wrote page data starting at address 0x10"),
        Err(e) => warn!("Failed to write page: {:?}", e),
    }

    // 等待写操作完成
    Timer::after_millis(10).await;

    // 逐个读取并验证写入的数据
    for i in 0..page_data.len() {
        match eeprom.read_byte(0x10 + i as u8).await {
            Ok(value) => {
                if value == page_data[i] {
                    info!(
                        "Verified data at address 0x{:02X}: 0x{:02X}",
                        0x10 + i as u8,
                        value
                    );
                } else {
                    warn!(
                        "Data mismatch at address 0x{:02X}: expected 0x{:02X}, got 0x{:02X}",
                        0x10 + i as u8,
                        page_data[i],
                        value
                    );
                }
            }
            Err(e) => warn!(
                "Failed to read byte at address 0x{:02X}: {:?}",
                0x10 + i as u8,
                e
            ),
        }
    }

    info!("EEPROM demo completed");
}
