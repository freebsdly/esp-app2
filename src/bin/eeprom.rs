// 在 main.rs 文件末尾添加以下代码

extern crate alloc;

use core::cell::RefCell;
use critical_section::Mutex;
use esp_hal::i2c::master::I2c;
use esp_hal::Blocking;

/// 24C02 EEPROM 驱动
pub struct At24C02<'a> {
    i2c: &'a Mutex<RefCell<Option<I2c<'a, Blocking>>>>,
    address: u8,
}

impl<'a> At24C02<'a> {
    pub fn new(i2c: &'a Mutex<RefCell<Option<I2c<'a, Blocking>>>>, address: u8) -> Self {
        Self { i2c, address }
    }

    /// 向指定地址写入一个字节的数据
    pub async fn write_byte(
        &mut self,
        addr: u8,
        data: u8,
    ) -> Result<(), esp_hal::i2c::master::Error> {
        critical_section::with(|cs| {
            let mut i2c_ref = self.i2c.borrow_ref_mut(cs);
            let i2c = i2c_ref.as_mut().unwrap();

            i2c.write(self.address, &[addr, data])
        })
    }

    /// 从指定地址读取一个字节的数据
    pub async fn read_byte(&mut self, addr: u8) -> Result<u8, esp_hal::i2c::master::Error> {
        let mut data = [0u8];

        critical_section::with(|cs| {
            let mut i2c_ref = self.i2c.borrow_ref_mut(cs);
            let i2c = i2c_ref.as_mut().unwrap();

            i2c.write_read(self.address, &[addr], &mut data)
        })?;

        Ok(data[0])
    }

    /// 向指定地址开始连续写入多个字节数据
    pub async fn write_page(
        &mut self,
        start_addr: u8,
        data: &[u8],
    ) -> Result<(), esp_hal::i2c::master::Error> {
        // 24C02每页8字节，确保不超过页面边界
        if data.len() > 8 || (start_addr % 8 + data.len() as u8) > 8 {
            return Err(esp_hal::i2c::master::Error::Timeout);
        }

        let mut buffer = alloc::vec![0u8; data.len() + 1];
        buffer[0] = start_addr;
        buffer[1..].copy_from_slice(data);

        critical_section::with(|cs| {
            let mut i2c_ref = self.i2c.borrow_ref_mut(cs);
            let i2c = i2c_ref.as_mut().unwrap();

            i2c.write(self.address, &buffer)
        })
    }
}
