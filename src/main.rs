//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use cortex_m::singleton;
use defmt::*;
use defmt_rtt as _;
use fugit::HertzU32;
use panic_probe as _;
use pio_proc::pio_file;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{Clock, ClockSource, ClocksManager, InitError},
    dma::{double_buffer, single_buffer, DMAExt},
    gpio::FunctionPio0,
    pac,
    pio::{Buffers, PIOBuilder, PIOExt, PinDir, ShiftDirection},
    pll::{common_configs::PLL_USB_48MHZ, setup_pll_blocking},
    sio::Sio,
    watchdog::Watchdog,
    xosc::setup_xosc_blocking,
};

mod rp2040_pll_settings_for_48khz_audio;

/// External high-speed crystal on the pico board is 12Mhz
const EXTERNAL_XTAL_FREQ_HZ: HertzU32 = HertzU32::from_raw(12_000_000u32);

/// RP2040の動作周波数
const RP2040_CLOCK_HZ: HertzU32 = HertzU32::from_raw(153_600_000u32);

/// PIOの動作周波数 15.36MHz(48kHz*64*5) I2SのMCLKをPIOで作るので48kHzの整数倍にする
const PIO_CLOCK_HZ: HertzU32 = HertzU32::from_raw(15_360_000u32);

/// PIOの分周比率の整数部分 RP2040動作周波数/PIO動作周波数
/// int + (frac/256)で分周する
const PIO_CLOCKDIV_INT: u16 = (RP2040_CLOCK_HZ.raw() / PIO_CLOCK_HZ.raw()) as u16;
/// PIOの分周比率の少数部分 Jitterを最小にするには0にするべき
const PIO_CLOCKDIV_FRAC: u8 = 0u8;

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

    // Configure PLL and clocks
    {
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
    }

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    delay.delay_ms(1); //不使用でワーニングを出さないために適当に1ms delayさせているだけ。意味はない。

    // ===============GPIO=================
    let pins = bsp::hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    //=================PIO=================
    // configure GPIO for PIO0.
    let mclk_pin = pins.gpio8.into_mode::<FunctionPio0>();
    let i2s_send_data_pin = pins.gpio9.into_mode::<FunctionPio0>();
    let i2s_send_sclk_pin = pins.gpio10.into_mode::<FunctionPio0>();
    let i2s_send_lrclk_pin = pins.gpio11.into_mode::<FunctionPio0>();

    let pio_i2s_mclk_output = pio_file!("./src/i2s.pio", select_program("mclk_output")).program;
    let pio_i2s_send_master_left =
        pio_file!("./src/i2s.pio", select_program("i2s_send_master_left")).program; //I2S送信のLch用 (Rch用と協調動作する)
    let pio_i2s_send_master_right =
        pio_file!("./src/i2s.pio", select_program("i2s_send_master_right")).program; //I2S送信のRch用

    // Initialize and start PIO
    let (mut pio, sm0, sm1, sm2, _sm3) = pac.PIO0.split(&mut pac.RESETS);
    let pio_i2s_mclk_output = pio.install(&pio_i2s_mclk_output).unwrap();
    let pio_i2s_send_master_left = pio.install(&pio_i2s_send_master_left).unwrap();
    let pio_i2s_send_master_right = pio.install(&pio_i2s_send_master_right).unwrap();

    let (mut sm0, _rx0, _tx0) = PIOBuilder::from_program(pio_i2s_mclk_output)
        .set_pins(mclk_pin.id().num, 1)
        .clock_divisor_fixed_point(PIO_CLOCKDIV_INT, PIO_CLOCKDIV_FRAC)
        .build(sm0);

    let (mut sm1, _rx1, mut tx1) = PIOBuilder::from_program(pio_i2s_send_master_left)
        .out_pins(i2s_send_data_pin.id().num, 1)
        .side_set_pin_base(i2s_send_sclk_pin.id().num)
        .clock_divisor_fixed_point(PIO_CLOCKDIV_INT, PIO_CLOCKDIV_FRAC)
        .out_shift_direction(ShiftDirection::Left) //左シフト I2SはMSB first
        .pull_threshold(32u8) //32bit bit depth LSB側が0埋めされた24bit, 16bitでも可
        .autopull(true)
        .buffers(Buffers::OnlyTx) // Rx FIFOは使わないので、その分をTx FIFOにjoin
        .build(sm1);

    let (mut sm2, _rx2, mut tx2) = PIOBuilder::from_program(pio_i2s_send_master_right)
        .out_pins(i2s_send_data_pin.id().num, 1)
        .clock_divisor_fixed_point(PIO_CLOCKDIV_INT, PIO_CLOCKDIV_FRAC)
        .out_shift_direction(ShiftDirection::Left) //左シフト I2SはMSB first
        .autopull(false) // No Auto Pull
        .pull_threshold(32u8) //32bit bit depth LSB側が0埋めされた24bit, 16bitでも可
        .buffers(Buffers::OnlyTx) // Rx FIFOは使わないので、その分をTx FIFOにjoin
        .build(sm2);

    // The GPIO pin needs to be configured as an output.
    sm0.set_pindirs([(mclk_pin.id().num, PinDir::Output)]);
    sm0.start();
    sm1.set_pindirs([
        (i2s_send_data_pin.id().num, PinDir::Output),
        (i2s_send_lrclk_pin.id().num, PinDir::Output),
        (i2s_send_sclk_pin.id().num, PinDir::Output),
    ]);
    //sm2はside-setは使わない
    sm2.set_pindirs([(i2s_send_data_pin.id().num, PinDir::Output)]);

    // ================DMA=================
    let dma_channels = pac.DMA.split(&mut pac.RESETS);
    let dma_buf = singleton!(: [u32; 4] = [0; 4]).unwrap(); //singleton! staticなバッファーを作る
    let dma_config = single_buffer::Config::new(dma_channels.ch0, dma_buf, tx1);

    // Start PIO
    let sm_group_i2s_send = sm1.with(sm2);
    sm_group_i2s_send.start();

    // PIO runs in background, independently from CPU
    loop {}
}
