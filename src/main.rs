//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use cic_fixed::CicDecimationFilter;
use cortex_m::singleton;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use fixed::types::I1F31;
use fugit::HertzU32;
use heapless::spsc::Queue;
use panic_probe as _;
use pio_proc::pio_file;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{Clock, ClockSource, ClocksManager, InitError},
    dma::{double_buffer, DMAExt},
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
const RP2040_CLOCK_HZ: HertzU32 = HertzU32::from_raw(230_400_000u32);

/// PIOの動作周波数 15.36MHz(48kHz*64*5) I2SのMCLKをPIOで作るので48kHzの整数倍にする
const PIO_CLOCK_HZ: HertzU32 = HertzU32::from_raw(15_360_000u32);

/// PIOの分周比率の整数部分 RP2040動作周波数/PIO動作周波数
/// int + (frac/256)で分周する
const PIO_CLOCKDIV_INT: u16 = (RP2040_CLOCK_HZ.raw() / PIO_CLOCK_HZ.raw()) as u16;
/// PIOの分周比率の少数部分 Jitterを最小にするには0にするべき
const PIO_CLOCKDIV_FRAC: u8 = 0u8;

/// バッファーサイズ（サンプル）
const BUFFER_SIZE: usize = 16;
const PDM_QUEUE_SIZE: usize = BUFFER_SIZE * 2;

//CICフィルターの出力（19bit）を32bit固定小数点に正規化するためのゲイン
// const GAIN: i32 = 2i32.pow(13);
const GAIN: i32 = 2i32.pow(16); //13bitのままだと音が小さいので16bitにしてみる

#[entry]
fn main() -> ! {
    info!("Program start");
    info!("BUFFER_SIZE: {=usize}", BUFFER_SIZE);
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
            rp2040_pll_settings_for_48khz_audio::SYS_PLL_CONFIG_230P4MHZ,
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
    delay.delay_ms(100); // PDMマイクのパワーアップシーケンスに50ms程度必要

    //=============================GPIO===============================
    let pins = bsp::hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // configure GPIO for PIO0.
    let mclk_pin = pins.gpio8.into_function::<FunctionPio0>();
    let i2s_send_data_pin = pins.gpio9.into_function::<FunctionPio0>();
    let i2s_send_sclk_pin = pins.gpio10.into_function::<FunctionPio0>();
    let i2s_send_lrclk_pin = pins.gpio11.into_function::<FunctionPio0>();
    let pdm_input_pin = pins.gpio12.into_function::<FunctionPio0>();
    let pdm_clock_output_pin = pins.gpio13.into_function::<FunctionPio0>();
    let mut pdm_pio_jump_pin = pins.gpio14.into_push_pull_output();
    pdm_pio_jump_pin.set_high().unwrap(); //PDM用PIOの起動直後はJUMP PINをHighにしてPDM clockを1.536MHzにし、1秒くらい経ったらLowにして3.072MHzにしてUltrasonic modeにする

    //=============================PIO===============================
    let pio_i2s_mclk_output = pio_file!("./src/i2s.pio", select_program("mclk_output")).program;
    let pio_i2s_send_master = pio_file!("./src/i2s.pio", select_program("i2s_send_master")).program;
    let pio_pdm = pio_file!(
        "./src/pdm.pio",
        select_program("pdm_stereo_SPH0641LU4H_ultrasonic_mode")
    )
    .program;

    // Initialize and start PIO
    let (mut pio, sm0, sm1, sm2, _sm3) = pac.PIO0.split(&mut pac.RESETS);
    let pio_i2s_mclk_output = pio.install(&pio_i2s_mclk_output).unwrap();
    let pio_i2s_send_master = pio.install(&pio_i2s_send_master).unwrap();
    let pio_pdm = pio.install(&pio_pdm).unwrap();

    // MCLK送信用PIOの設定
    let (mut sm0, _rx0, _tx0) = PIOBuilder::from_program(pio_i2s_mclk_output)
        .set_pins(mclk_pin.id().num, 1)
        .clock_divisor_fixed_point(PIO_CLOCKDIV_INT, PIO_CLOCKDIV_FRAC)
        .build(sm0);

    // I2S送信用PIOの設定
    let (mut sm1, _rx1, tx1) = PIOBuilder::from_program(pio_i2s_send_master)
        .out_pins(i2s_send_data_pin.id().num, 1)
        .side_set_pin_base(i2s_send_sclk_pin.id().num)
        .clock_divisor_fixed_point(PIO_CLOCKDIV_INT, PIO_CLOCKDIV_FRAC)
        .out_shift_direction(ShiftDirection::Left) //左シフト I2SはMSB first
        .autopull(true)
        .pull_threshold(32u8) //Bit-depth: 32bit
        .buffers(Buffers::OnlyTx) // Rx FIFOは使わないので、その分をTx FIFOにjoin
        .build(sm1);

    // PDM用PIOの設定
    let (mut sm2, rx2, _tx1) = PIOBuilder::from_program(pio_pdm)
        .in_pin_base(pdm_input_pin.id().num)
        .side_set_pin_base(pdm_clock_output_pin.id().num)
        .jmp_pin(pdm_pio_jump_pin.id().num) // PIO起動直後はJUMP PINをHighにしてPDM clockを1.536MHzにし、1秒くらい経ったらLowにして3.072MHzにする
        .clock_divisor_fixed_point(PIO_CLOCKDIV_INT, PIO_CLOCKDIV_FRAC)
        .in_shift_direction(ShiftDirection::Left) //左シフト
        .autopush(true)
        .push_threshold(32u8) //Bit-depth: 32bit
        .buffers(Buffers::OnlyRx) // Tx FIFOは使わないので、その分をRx FIFOにjoin
        .build(sm2);

    // The GPIO pin needs to be configured as an output.
    sm0.set_pindirs([(mclk_pin.id().num, PinDir::Output)]);
    sm0.start(); // Start MCLK PIO (MCLKの位相はI2Sと合う必要はないので適当なタイミングでスタートして良い)。
    sm1.set_pindirs([
        (i2s_send_data_pin.id().num, PinDir::Output),
        (i2s_send_lrclk_pin.id().num, PinDir::Output),
        (i2s_send_sclk_pin.id().num, PinDir::Output),
    ]);
    sm2.set_pindirs([
        (pdm_input_pin.id().num, PinDir::Input),
        (pdm_clock_output_pin.id().num, PinDir::Output),
    ]);

    //=============================DMA===============================
    // I2S用DMA設定
    // tx_buf1とtx_buf2でダブルバッファリングしてI2SのPIOのFIFOへ転送する
    let dma_channels = pac.DMA.split(&mut pac.RESETS);
    let i2s_tx_buf1 = singleton!(: [u32; BUFFER_SIZE*2] = [12345; BUFFER_SIZE*2]).unwrap(); //staticなバッファーを作る
    let i2s_tx_buf2 = singleton!(: [u32; BUFFER_SIZE*2] = [123; BUFFER_SIZE*2]).unwrap(); //staticなバッファーを作る
    let i2s_dma_config =
        double_buffer::Config::new((dma_channels.ch0, dma_channels.ch1), i2s_tx_buf1, tx1);
    let i2s_tx_transfer = i2s_dma_config.start(); //転送開始
    let mut i2s_tx_transfer = i2s_tx_transfer.read_next(i2s_tx_buf2);

    // PDM用DMA設定
    let pdm_rx_buf1 = singleton!(: [u32; BUFFER_SIZE*4] = [0; BUFFER_SIZE*4]).unwrap(); //staticなバッファーを作る
    let pdm_rx_buf2 = singleton!(: [u32; BUFFER_SIZE*4] = [0; BUFFER_SIZE*4]).unwrap(); //staticなバッファーを作る
    let pdm_dma_config =
        double_buffer::Config::new((dma_channels.ch2, dma_channels.ch3), rx2, pdm_rx_buf1);
    let pdm_rx_transfer = pdm_dma_config.start(); //転送開始
    let mut pdm_rx_transfer = pdm_rx_transfer.write_next(pdm_rx_buf2);

    let sm_group_i2s_pdm = sm1.with(sm2); //I2SとPDMのPIOを同時にスタートさせるためにグループ化する

    let mut l_pdm_queue: Queue<I1F31, PDM_QUEUE_SIZE> = Queue::new();
    let mut r_pdm_queue: Queue<I1F31, PDM_QUEUE_SIZE> = Queue::new();
    let mut l_cic = CicDecimationFilter::<64, 3>::new(); //CICフィルターの初期化
    let mut r_cic = CicDecimationFilter::<64, 3>::new(); //CICフィルターの初期化
    const INPUT_BITS: u32 = 1; //PDMなので1bit
    let bit_growth = l_cic.bit_growth(); //CICフィルターによって増加するBit数
    let output_bits = INPUT_BITS + bit_growth; //CICフィルターから出力されるBit数
    info!("Bit growth of CIC: {}bits", bit_growth);
    info!("Output bits of CIC: {}bits", output_bits);

    //PDM QueueをBUFFER_SIZE分だけ0埋めして初期化
    {
        for _ in 0..BUFFER_SIZE {
            l_pdm_queue.enqueue(I1F31::ZERO).unwrap();
            r_pdm_queue.enqueue(I1F31::ZERO).unwrap();
        }
    }

    //I2SとPDMのPIOスタート
    sm_group_i2s_pdm.start();
    delay.delay_ms(1000);
    pdm_pio_jump_pin.set_low().unwrap(); //PDM用PIOの起動直後はJUMP PINをHighにしてPDM clockを1.536MHzにし、1秒くらい経ったらLowにして3.072MHzにしてUltrasonic modeにする

    loop {
        // I2SのDMA転送が終わったら即座にもう片方のバッファーを転送する
        if i2s_tx_transfer.is_done() {
            let (next_tx_buf, next_tx_transfer) = i2s_tx_transfer.wait();

            // info!("I2S done: {}", next_tx_buf.len());

            // 信号処理的な
            for (i, e) in next_tx_buf.iter_mut().enumerate() {
                if i % 2 == 0 {
                    // Lch
                    let l = l_pdm_queue.dequeue().unwrap_or(I1F31::ZERO);
                    // 信号処理をするならここでやる
                    let sample = l.to_bits() * GAIN;
                    *e = sample as u32;
                } else {
                    // Rch
                    let r = r_pdm_queue.dequeue().unwrap_or(I1F31::ZERO);
                    // 信号処理をするならここでやる
                    let sample = r.to_bits() * GAIN;
                    *e = sample as u32;
                }
            }

            i2s_tx_transfer = next_tx_transfer.read_next(next_tx_buf);
        }

        if pdm_rx_transfer.is_done() {
            let (rx_buf, next_rx_transfer) = pdm_rx_transfer.wait();

            for e in rx_buf.iter() {
                //上位ビットから順に処理する
                let cic_input_value: i32 = if bit_bang(*e, 31) { 1i32 } else { -1i32 };
                if let Some(v) = l_cic.filter(cic_input_value) {
                    l_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }
                let cic_input_value: i32 = if bit_bang(*e, 30) { 1i32 } else { -1i32 };
                if let Some(v) = r_cic.filter(cic_input_value) {
                    r_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }

                let cic_input_value: i32 = if bit_bang(*e, 29) { 1i32 } else { -1i32 };
                if let Some(v) = l_cic.filter(cic_input_value) {
                    l_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }
                let cic_input_value: i32 = if bit_bang(*e, 28) { 1i32 } else { -1i32 };
                if let Some(v) = r_cic.filter(cic_input_value) {
                    r_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }

                let cic_input_value: i32 = if bit_bang(*e, 27) { 1i32 } else { -1i32 };
                if let Some(v) = l_cic.filter(cic_input_value) {
                    l_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }
                let cic_input_value: i32 = if bit_bang(*e, 26) { 1i32 } else { -1i32 };
                if let Some(v) = r_cic.filter(cic_input_value) {
                    r_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }

                let cic_input_value: i32 = if bit_bang(*e, 25) { 1i32 } else { -1i32 };
                if let Some(v) = l_cic.filter(cic_input_value) {
                    l_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }
                let cic_input_value: i32 = if bit_bang(*e, 24) { 1i32 } else { -1i32 };
                if let Some(v) = r_cic.filter(cic_input_value) {
                    r_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }

                let cic_input_value: i32 = if bit_bang(*e, 23) { 1i32 } else { -1i32 };
                if let Some(v) = l_cic.filter(cic_input_value) {
                    l_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }
                let cic_input_value: i32 = if bit_bang(*e, 22) { 1i32 } else { -1i32 };
                if let Some(v) = r_cic.filter(cic_input_value) {
                    r_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }

                let cic_input_value: i32 = if bit_bang(*e, 21) { 1i32 } else { -1i32 };
                if let Some(v) = l_cic.filter(cic_input_value) {
                    l_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }
                let cic_input_value: i32 = if bit_bang(*e, 20) { 1i32 } else { -1i32 };
                if let Some(v) = r_cic.filter(cic_input_value) {
                    r_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }

                let cic_input_value: i32 = if bit_bang(*e, 19) { 1i32 } else { -1i32 };
                if let Some(v) = l_cic.filter(cic_input_value) {
                    l_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }
                let cic_input_value: i32 = if bit_bang(*e, 18) { 1i32 } else { -1i32 };
                if let Some(v) = r_cic.filter(cic_input_value) {
                    r_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }

                let cic_input_value: i32 = if bit_bang(*e, 17) { 1i32 } else { -1i32 };
                if let Some(v) = l_cic.filter(cic_input_value) {
                    l_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }
                let cic_input_value: i32 = if bit_bang(*e, 16) { 1i32 } else { -1i32 };
                if let Some(v) = r_cic.filter(cic_input_value) {
                    r_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }

                let cic_input_value: i32 = if bit_bang(*e, 15) { 1i32 } else { -1i32 };
                if let Some(v) = l_cic.filter(cic_input_value) {
                    l_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }
                let cic_input_value: i32 = if bit_bang(*e, 14) { 1i32 } else { -1i32 };
                if let Some(v) = r_cic.filter(cic_input_value) {
                    r_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }

                let cic_input_value: i32 = if bit_bang(*e, 13) { 1i32 } else { -1i32 };
                if let Some(v) = l_cic.filter(cic_input_value) {
                    l_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }
                let cic_input_value: i32 = if bit_bang(*e, 12) { 1i32 } else { -1i32 };
                if let Some(v) = r_cic.filter(cic_input_value) {
                    r_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }

                let cic_input_value: i32 = if bit_bang(*e, 11) { 1i32 } else { -1i32 };
                if let Some(v) = l_cic.filter(cic_input_value) {
                    l_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }
                let cic_input_value: i32 = if bit_bang(*e, 10) { 1i32 } else { -1i32 };
                if let Some(v) = r_cic.filter(cic_input_value) {
                    r_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }

                let cic_input_value: i32 = if bit_bang(*e, 9) { 1i32 } else { -1i32 };
                if let Some(v) = l_cic.filter(cic_input_value) {
                    l_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }
                let cic_input_value: i32 = if bit_bang(*e, 8) { 1i32 } else { -1i32 };
                if let Some(v) = r_cic.filter(cic_input_value) {
                    r_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }

                let cic_input_value: i32 = if bit_bang(*e, 7) { 1i32 } else { -1i32 };
                if let Some(v) = l_cic.filter(cic_input_value) {
                    l_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }
                let cic_input_value: i32 = if bit_bang(*e, 6) { 1i32 } else { -1i32 };
                if let Some(v) = r_cic.filter(cic_input_value) {
                    r_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }

                let cic_input_value: i32 = if bit_bang(*e, 5) { 1i32 } else { -1i32 };
                if let Some(v) = l_cic.filter(cic_input_value) {
                    l_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }
                let cic_input_value: i32 = if bit_bang(*e, 4) { 1i32 } else { -1i32 };
                if let Some(v) = r_cic.filter(cic_input_value) {
                    r_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }

                let cic_input_value: i32 = if bit_bang(*e, 3) { 1i32 } else { -1i32 };
                if let Some(v) = l_cic.filter(cic_input_value) {
                    l_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }
                let cic_input_value: i32 = if bit_bang(*e, 2) { 1i32 } else { -1i32 };
                if let Some(v) = r_cic.filter(cic_input_value) {
                    r_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }

                let cic_input_value: i32 = if bit_bang(*e, 1) { 1i32 } else { -1i32 };
                if let Some(v) = l_cic.filter(cic_input_value) {
                    l_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }
                let cic_input_value: i32 = if bit_bang(*e, 0) { 1i32 } else { -1i32 };
                if let Some(v) = r_cic.filter(cic_input_value) {
                    r_pdm_queue.enqueue(I1F31::from_bits(v)).unwrap();
                }
            }

            pdm_rx_transfer = next_rx_transfer.write_next(rx_buf);
        }
    }
}

/// 32bitの値の指定したビットが1か0かを返す
/// # Arguments
/// * `v` - 32bitの値
/// * `index` - 0~31のビット位置（LSBが0）
#[inline]
const fn bit_bang(v: u32, index: u8) -> bool {
    v & (1 << index) > 0
}
