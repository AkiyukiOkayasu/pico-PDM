//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

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
    clocks::{Clock, ClockSource, ClocksManager, InitError},
    pac,
    pll::{common_configs::PLL_USB_48MHZ, setup_pll_blocking, PLLConfig},
    sio::Sio,
    watchdog::Watchdog,
    xosc::setup_xosc_blocking,
};

use fugit::HertzU32;

mod rp2040_pll_settings_for_48khz_audio;

/// External high-speed crystal on the pico board is 12Mhz
const EXTERNAL_XTAL_FREQ_HZ: HertzU32 = HertzU32::from_raw(12_000_000u32);

/// RP2040の動作周波数 76.8MHz
const RP2040_CLOCK_HZ: HertzU32 = HertzU32::from_raw(76_800_000u32);

/// PIOの動作周波数 15.36MHz(48kHz*64*5) I2SのMCLKをPIOで作るので48kHzの整数倍にする
const PIO_CLOCK_HZ: HertzU32 = HertzU32::from_raw(15_360_000u32);

/// PIOの分周比率 15=RP2040動作周波数/PIO動作周波数
const PIO_CLOCKDIV_INT: u32 = RP2040_CLOCK_HZ.raw() / PIO_CLOCK_HZ.raw();

/// RP2040を76.8MHzで動作させるためのPLL設定
/// $PICO_SDK/src/rp2_common/hardware_clocks/scripts/vcocalc.py
const SYS_PLL_CONFIG_76800KHZ: PLLConfig = PLLConfig {
    vco_freq: HertzU32::MHz(1536),
    refdiv: 1,
    post_div1: 5,
    post_div2: 4,
};

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // Enable the xosc
    let xosc = setup_xosc_blocking(pac.XOSC, EXTERNAL_XTAL_FREQ_HZ)
        .map_err(InitError::XoscErr)
        .ok()
        .unwrap();

    // Start tick in watchdog
    watchdog.enable_tick_generation((EXTERNAL_XTAL_FREQ_HZ.raw() / 1_000_000) as u8);

    let mut clocks = ClocksManager::new(pac.CLOCKS);

    // Configure PLLs
    //                   REF     FBDIV VCO            POSTDIV
    // PLL SYS: 12 / 1 = 12MHz * 125 = 1500MHZ / 6 / 2 = 125MHz
    // PLL USB: 12 / 1 = 12MHz * 40  = 480 MHz / 5 / 2 =  48MHz
    let pll_sys = setup_pll_blocking(
        pac.PLL_SYS,
        xosc.operating_frequency(),
        rp2040_pll_settings_for_48khz_audio::SYS_PLL_CONFIG_153P6MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .map_err(InitError::PllError)
    .ok()
    .unwrap();

    let pll_usb = setup_pll_blocking(
        pac.PLL_USB,
        xosc.operating_frequency(),
        PLL_USB_48MHZ,
        &mut clocks,
        &mut pac.RESETS,
    )
    .map_err(InitError::PllError)
    .ok()
    .unwrap();

    // Configure clocks
    // CLK_REF = XOSC (12MHz) / 1 = 12MHz
    clocks
        .reference_clock
        .configure_clock(&xosc, xosc.get_freq())
        .map_err(InitError::ClockError)
        .ok()
        .unwrap();

    // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
    clocks
        .system_clock
        .configure_clock(&pll_sys, pll_sys.get_freq())
        .map_err(InitError::ClockError)
        .ok()
        .unwrap();

    // CLK USB = PLL USB (48MHz) / 1 = 48MHz
    clocks
        .usb_clock
        .configure_clock(&pll_usb, pll_usb.get_freq())
        .map_err(InitError::ClockError)
        .ok()
        .unwrap();

    // CLK ADC = PLL USB (48MHZ) / 1 = 48MHz
    clocks
        .adc_clock
        .configure_clock(&pll_usb, pll_usb.get_freq())
        .map_err(InitError::ClockError)
        .ok()
        .unwrap();

    // CLK RTC = PLL USB (48MHz) / 1024 = 46875Hz
    clocks
        .rtc_clock
        .configure_clock(&pll_usb, HertzU32::from_raw(46875u32))
        .map_err(InitError::ClockError)
        .ok()
        .unwrap();

    // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
    // Normally choose clk_sys or clk_usb
    clocks
        .peripheral_clock
        .configure_clock(&clocks.system_clock, clocks.system_clock.freq())
        .map_err(InitError::ClockError)
        .ok()
        .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead. If you have
    // a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here.
    let mut led_pin = pins.led.into_push_pull_output();

    loop {
        info!("on!");
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        info!("off!");
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

// End of file
