#![allow(non_camel_case_types)]
#![allow(non_camel_case_types)]

/// Pressure rate
#[derive(Copy, Clone, Debug)]
pub enum PressureRate {
    //  See P.9 of manual
    _1_SPS = 0b000,
    _2_SPS = 0b001,
    _4_SPS = 0b010,
    _8_SPS = 0b011,
    _16_SPS = 0b100,
    _32_SPS = 0b101,
    _64_SPS = 0b110,
    _128_SPS = 0b111,
}

impl PressureRate {
    pub fn val(self) -> u8 {
        self as u8
    }
}

impl Default for PressureRate {
    fn default() -> Self {
        PressureRate::_1_SPS
    }
}

/// Pressure resolution
#[derive(Copy, Clone, Debug)]
pub enum PressureResolution {
    // See section 8.3, PM_RC field
    _1_SAMPLES = 0b000,
    _2_SAMPLES = 0b001,
    _4_SAMPLES = 0b010,
    _8_SAMPLES = 0b011,
    _16_SAMPLES = 0b100,
    _32_SAMPLES = 0b101,
    _64_SAMPLES = 0b110,
    _128_SAMPLES = 0b111, // Available for measurements in background mode only
}

impl PressureResolution {
    pub fn val(self) -> u8 {
        self as u8
    }

    #[allow(non_snake_case)]
    pub fn get_kP_value(self) -> f32 {
        // See Section 4.9.3 Sec. "Compensation Scale Factors", Table 9. Col. "Scale Factor column"
        match self {
            PressureResolution::_1_SAMPLES => 524_288_f32,
            PressureResolution::_2_SAMPLES => 1_572_864_f32,
            PressureResolution::_4_SAMPLES => 3_670_016_f32,
            PressureResolution::_8_SAMPLES => 7_864_320_f32,
            PressureResolution::_16_SAMPLES => 253_952_f32,
            PressureResolution::_32_SAMPLES => 516_096_f32,
            PressureResolution::_64_SAMPLES => 1_040_384_f32,
            PressureResolution::_128_SAMPLES => 2_088_960_f32,
        }
    }
}

impl Default for PressureResolution {
    fn default() -> Self {
        PressureResolution::_1_SAMPLES
    }
}

/// Temperature rate
#[derive(Copy, Clone, Debug)]
pub enum TemperatureRate {
    // See section 8.4 Temperature Configuration, TMP_RATE
    _1_SPS = 0b000,
    _2_SPS = 0b001,
    _4_SPS = 0b010,
    _8_SPS = 0b011,
    _16_SPS = 0b100,
    _32_SPS = 0b101,
    _64_SPS = 0b110,
    _128_SPS = 0b111, // Applicable for measurements in Background mode only
}

impl TemperatureRate {
    pub fn val(self) -> u8 {
        self as u8
    }
}

impl Default for TemperatureRate {
    fn default() -> Self {
        TemperatureRate::_1_SPS
    }
}

/// Temperature resolution
#[derive(Copy, Clone, Debug)]
pub enum TemperatureResolution {
    // See section 8.4 Temperature Configuration, TMP_PRC
    _1_SAMPLES = 0b000,
    _2_SAMPLES = 0b001,
    _4_SAMPLES = 0b010,
    _8_SAMPLES = 0b011,
    _16_SAMPLES = 0b100,
    _32_SAMPLES = 0b101,
    _64_SAMPLES = 0b110,
    _128_SAMPLES = 0b111,
}

impl TemperatureResolution {
    pub fn val(self) -> u8 {
        self as u8
    }

    #[allow(non_snake_case)]
    pub fn get_kt_value(self) -> f32 {
        // See Section 4.9.3 Sec. "Compensation Scale Factors", Table 9. Col. "Scale Factor column"
        match self {
            TemperatureResolution::_1_SAMPLES => 524_288_f32,
            TemperatureResolution::_2_SAMPLES => 1_572_864_f32,
            TemperatureResolution::_4_SAMPLES => 3_670_016_f32,
            TemperatureResolution::_8_SAMPLES => 7_864_320_f32,
            TemperatureResolution::_16_SAMPLES => 253_952_f32,
            TemperatureResolution::_32_SAMPLES => 516_096_f32,
            TemperatureResolution::_64_SAMPLES => 1_040_384_f32,
            TemperatureResolution::_128_SAMPLES => 2_088_960_f32,
        }
    }
}

impl Default for TemperatureResolution {
    fn default() -> Self {
        TemperatureResolution::_1_SAMPLES
    }
}

/// Configuration struct
#[derive(Copy, Clone, Debug)]
pub struct Config {
    pub(crate) pres_rate: Option<PressureRate>,
    pub(crate) pres_res: Option<PressureResolution>,
    pub(crate) temp_rate: Option<TemperatureRate>,
    pub(crate) temp_res: Option<TemperatureResolution>,
    pub(crate) int_hl: bool,
    pub(crate) int_fifo: bool,
    pub(crate) int_temp: bool,
    pub(crate) int_pres: bool,
    pub(crate) temp_shift: bool,
    pub(crate) pres_shift: bool,
    pub(crate) fifo_enable: bool,
    pub(crate) spi_mode: bool,
}

impl Config {
    /// Creates a new configuration object with default values
    pub fn new() -> Self {
        Config {
            pres_rate: None,
            pres_res: None,
            temp_rate: None,
            temp_res: None,
            int_hl: false,
            int_fifo: false,
            int_temp: false,
            int_pres: false,
            temp_shift: false,
            pres_shift: false,
            fifo_enable: false,
            spi_mode: false,
        }
    }

    pub fn pres_rate(&mut self, rate: PressureRate) -> &mut Self {
        self.pres_rate = Some(rate);
        self
    }

    pub fn pres_res(&mut self, res: PressureResolution) -> &mut Self {
        self.pres_res = Some(res);
        self
    }

    pub fn temp_rate(&mut self, rate: TemperatureRate) -> &mut Self {
        self.temp_rate = Some(rate);
        self
    }

    pub fn temp_res(&mut self, res: TemperatureResolution) -> &mut Self {
        self.temp_res = Some(res);
        self
    }

    /// Interrupt (on SDO pin) active level
    pub fn int_hl(&mut self, int_on_sdo_pin: bool) -> &mut Self {
        self.int_hl = int_on_sdo_pin;
        self
    }

    pub fn int_fifo(&mut self, int_on_fifo: bool) -> &mut Self {
        self.int_fifo = int_on_fifo;
        self
    }

    pub fn int_temp(&mut self, int_on_temp: bool) -> &mut Self {
        self.int_temp = int_on_temp;
        self
    }

    pub fn int_pres(&mut self, int_on_pres: bool) -> &mut Self {
        self.int_pres = int_on_pres;
        self
    }

    /// Set temperature result bit-shift, Must be set to true when oversampling rate > 8 times
    pub fn temp_shift(&mut self, temp_shift_enable: bool) -> &mut Self {
        self.temp_shift = temp_shift_enable;
        self
    }

    /// Set pressure result bit-shift, Must be set to true when oversampling rate > 8 times
    pub fn pres_shift(&mut self, pres_shift_enable: bool) -> &mut Self {
        self.pres_shift = pres_shift_enable;
        self
    }

    /// Set fifo options
    pub fn fifo(&mut self, interrupt_on_full: bool, enable: bool) -> &mut Self {
        self.int_fifo = interrupt_on_full;
        self.fifo_enable = enable;
        self
    }

    /// Set SPI mode (false -> 4 wire, true -> 3 wire interface)
    pub fn spi_mode(&mut self, three_wire: bool) -> &mut Self {
        self.spi_mode = three_wire;
        self
    }
}
