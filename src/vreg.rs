//! Voltage regulation API
//!
//! See [Chapter 2, Section 10](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf) of the datasheet for more details
//!
//! ## Usage
//! ```no_run
//! use rp2040_hal::{vreg::{VregVoltage, vreg_set_voltage}, pac};
//! let mut pac = pac::Peripherals::take().unwrap();
//! vreg_set_voltage(&mut pac.VREG_AND_CHIP_RESET, VregVoltage::Voltage1_20);
//! ```

use crate::pac::VREG_AND_CHIP_RESET;

/// Possible voltage values that can be applied to the regulator
#[allow(dead_code)]
pub enum VregVoltage {
    /// 0.85V
    Voltage0_85 = 0b0110,
    /// 0.90V   
    Voltage0_90 = 0b0111,
    /// 0.95V
    Voltage0_95 = 0b1000,
    /// 1.00V
    Voltage1_00 = 0b1001,
    /// 1.05V
    Voltage1_05 = 0b1010,
    /// 1.10V (default)
    Voltage1_10 = 0b1011,
    /// 1.15V
    Voltage1_15 = 0b1100,
    /// 1.20V
    Voltage1_20 = 0b1101,
    /// 1.25V
    Voltage1_25 = 0b1110,
    /// 1.30V
    Voltage1_30 = 0b1111,
}

/// Set voltage
///
/// voltage  The voltage (from enumeration \ref vreg_voltage) to apply to the voltage regulator
pub fn vreg_set_voltage(vreg_dev: &mut VREG_AND_CHIP_RESET, voltage: VregVoltage) {
    vreg_dev
        .vreg
        .write(|w| unsafe { w.vsel().bits(voltage as u8) });
}

/// Get voltage
/// TODO u8からVregVoltageに変換する
pub fn vreg_get_voltage(vreg_dev: &mut VREG_AND_CHIP_RESET) -> u8 {
    vreg_dev.vreg.read().vsel().bits()
}
