// ========================
// XL9555 寄存器定义
// ========================
pub const XL9555_ADDR: u8 = 0x20; // 7-bit I2C 地址

// 寄存器地址
pub mod regsisters {
    pub const INPUT_PORT_0: u8 = 0;
    pub const INPUT_PORT_1: u8 = 1;
    // pub const OUTPUT_PORT_0: u8 = 2;
    // pub const OUTPUT_PORT_1: u8 = 3;
    // pub const INVERSION_PORT_0: u8 = 4;
    // pub const INVERSION_PORT_1: u8 = 5;
    // pub const CONFIG_PORT_0: u8 = 6;
    // pub const CONFIG_PORT_1: u8 = 7;
}

// IO 位定义 (P0: bit 0~7, P1: bit 8~15)
pub mod io_bits {
    // pub const AP_INT_IO: u16 = 0x0001; // P0.0
    // pub const QMA_INT_IO: u16 = 0x0002; // P0.1
    // pub const SPK_EN_IO: u16 = 0x0004; // P0.2
    // pub const BEEP_IO: u16 = 0x0008; // P0.3
    // pub const OV_PWDN_IO: u16 = 0x0010; // P0.4
    // pub const OV_RESET_IO: u16 = 0x0020; // P0.5
    // pub const GBC_LED_IO: u16 = 0x0040; // P0.6
    // pub const GBC_KEY_IO: u16 = 0x0080; // P0.7
    // pub const LCD_BL_IO: u16 = 0x0100; // P1.0
    // pub const CT_RST_IO: u16 = 0x0200; // P1.1
    // pub const SLCD_RST_IO: u16 = 0x0400; // P1.2
    // pub const SLCD_PWR_IO: u16 = 0x0800; // P1.3
    pub const KEY3_IO: u16 = 0x1000; // P1.4
    pub const KEY2_IO: u16 = 0x2000; // P1.5
    pub const KEY1_IO: u16 = 0x4000; // P1.6
    pub const KEY0_IO: u16 = 0x8000; // P1.7
}
