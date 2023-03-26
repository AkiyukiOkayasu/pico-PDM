//! RP2040_PLL_Settings for 48kHz audio
//!
//! 48kHzの整数倍かつ、jitterの少ない（low_vcoではない）設定をまとめたもの

use fugit::HertzU32;
use rp_pico::hal::pll::PLLConfig;

/// RP2040を76.8MHzで動作させるためのPLL設定
/// $PICO_SDK/src/rp2_common/hardware_clocks/scripts/vcocalc.py
pub const SYS_PLL_CONFIG_76P8MHZ: PLLConfig = PLLConfig {
    vco_freq: HertzU32::MHz(1536),
    refdiv: 1,
    post_div1: 5,
    post_div2: 4,
};

/// RP2040を115.2MHzで動作させるためのPLL設定
/// $PICO_SDK/src/rp2_common/hardware_clocks/scripts/vcocalc.py
pub const SYS_PLL_CONFIG_115P2MHZ: PLLConfig = PLLConfig {
    vco_freq: HertzU32::MHz(1152),
    refdiv: 1,
    post_div1: 5,
    post_div2: 2,
};

/// RP2040を153.6MHzで動作させるためのPLL設定
/// $PICO_SDK/src/rp2_common/hardware_clocks/scripts/vcocalc.py
pub const SYS_PLL_CONFIG_153P6MHZ: PLLConfig = PLLConfig {
    vco_freq: HertzU32::MHz(1536),
    refdiv: 1,
    post_div1: 5,
    post_div2: 2,
};

/// RP2040を230.4MHzで動作させるためのPLL設定
/// $PICO_SDK/src/rp2_common/hardware_clocks/scripts/vcocalc.py
pub const SYS_PLL_CONFIG_230P4MHZ: PLLConfig = PLLConfig {
    vco_freq: HertzU32::MHz(1152),
    refdiv: 1,
    post_div1: 5,
    post_div2: 1,
};

/// RP2040を288MHzで動作させるためのPLL設定
/// $PICO_SDK/src/rp2_common/hardware_clocks/scripts/vcocalc.py
pub const SYS_PLL_CONFIG_288MHZ: PLLConfig = PLLConfig {
    vco_freq: HertzU32::MHz(1440),
    refdiv: 1,
    post_div1: 5,
    post_div2: 1,
};

/// RP2040を307.2MHzで動作させるためのPLL設定
/// $PICO_SDK/src/rp2_common/hardware_clocks/scripts/vcocalc.py
/// クロックが速すぎてQSPI Flashの調整をしないといけないかもしれない
pub const SYS_PLL_CONFIG_307P2MHZ: PLLConfig = PLLConfig {
    vco_freq: HertzU32::MHz(1536),
    refdiv: 1,
    post_div1: 5,
    post_div2: 1,
};

/// RP2040を384MHzで動作させるためのPLL設定
/// $PICO_SDK/src/rp2_common/hardware_clocks/scripts/vcocalc.py
/// クロックが速すぎてQSPI Flashの調整をしないといけないかもしれない
pub const SYS_PLL_CONFIG_384MHZ: PLLConfig = PLLConfig {
    vco_freq: HertzU32::MHz(1536),
    refdiv: 1,
    post_div1: 4,
    post_div2: 1,
};
