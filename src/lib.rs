//!
//! DSP310 embedded-hal I2C driver crate
//!
//! A platform agnostic driver to interface with the DSP310 barometric pressure & temp sensor.
//! This driver uses I2C via [embedded-hal]. Note that the DSP310 also supports SPI, however that
//! is not (yet) implemented in this driver.
//!
//! [embedded-hal]: https://docs.rs/embedded-hal

#![no_std]

mod config;
mod register;

use embedded_hal as hal;
use hal::blocking::i2c;

pub use config::*;
pub use register::Register;

/// DPS310 Product ID <https://www.infineon.com/dgdl/Infineon-DPS310-DataSheet-v01_01-EN.pdf?fileId=5546d462576f34750157750826c42242>, P. 25
const PRODUCT_ID: u8 = 0x10;

/// CalibrationCoeffs used to calculate calibration temperature and pressure values
#[derive(Debug, Clone, Copy)]
#[allow(non_snake_case)]
pub struct CalbrationCoeffs {
    // See Sec. 4.9.1 && Sec. 8.11 for declaration/definition
    pub C0: i32,
    pub C1: i32,
    pub C00: i32,
    pub C01: i32,
    pub C10: i32,
    pub C11: i32,
    pub C20: i32,
    pub C21: i32,
    pub C30: i32,
}

impl CalbrationCoeffs {
    fn default() -> Self {
        Self {
            C0: 0,
            C1: 0,
            C00: 0,
            C01: 0,
            C10: 0,
            C11: 0,
            C20: 0,
            C21: 0,
            C30: 0,
        }
    }
}

pub struct DPS310<I2C> {
    i2c: I2C,
    address: u8,
    coeffs: CalbrationCoeffs,
    pres_res: PressureResolution,
    temp_res: TemperatureResolution,
}

impl<I2C, E> DPS310<I2C>
where
    I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E>,
{
    pub fn new(i2c: I2C, address: u8, config: &Config) -> Result<Self, E> {
        let mut dsp310 = Self {
            i2c,
            address,
            coeffs: CalbrationCoeffs::default(),
            pres_res: config.pres_res.unwrap_or_default(),
            temp_res: config.temp_res.unwrap_or_default(),
        };

        // dsp310.reset().ok();

        let id: u8 = dsp310.get_product_id()?;
        if id != PRODUCT_ID {}

        dsp310.read_calibration_coefficients().ok();
        dsp310.apply_config(config)?;
        dsp310.standby().ok();

        Ok(dsp310)
    }

    fn apply_config(&mut self, config: &Config) -> Result<(), E> {
        // Sec 8.3 PRS_CFG register bits, PM_RATE & PM_PRC fields
        let prs_cfg = self.read_reg(Register::PRS_CFG)?;

        // keep first bit from current pressure config as that is reserved
        let new_prs_cfg = (prs_cfg >> 7) << 7
            | ((config.pres_rate.unwrap_or_default() as u8) << 4)
            | (config.pres_res.unwrap_or_default() as u8);

        self.write_reg(Register::PRS_CFG, new_prs_cfg)?;

        // Sec 8.4 Temperature Configuration (TMP_CFG), p.31 / Sec 8.11

        let temp_rate = config.temp_rate.unwrap_or_default() as u8;
        let pressure_rate = config.pres_rate.unwrap_or_default() as u8;
        let mut tmp_ext = self.read_reg(Register::TMP_COEF_SRCE)?;

        tmp_ext = (tmp_ext >> 7) << 7;
        tmp_ext |= temp_rate << 4 | (config.temp_res.unwrap_or_default() as u8);

        self.write_reg(Register::TEMP_CFG, tmp_ext)?;

        // if sampling rate > 8, must set the shift bit
        let shift_bit =
            temp_rate > TemperatureRate::_8_SPS as u8 || pressure_rate > PressureRate::_8_SPS as u8;

        // Sec 8.6 Interrupt and FIFO configuration (CFG_REG), p.39
        let cfg = ((config.int_hl as u8) << 7)
            | ((config.int_fifo as u8) << 6)
            | ((config.int_temp as u8) << 5)
            | ((config.int_pres as u8) << 4)
            | (((config.temp_shift | shift_bit) as u8) << 3)
            | (((config.pres_shift | shift_bit) as u8) << 2)
            | ((config.fifo_enable as u8) << 1)
            | ((config.spi_mode as u8) << 0);

        self.write_reg(Register::CFG_REG, cfg)?;

        Ok(())
    }

    /// Set measurement mode to `idle`
    fn standby(&mut self) -> Result<(), E> {
        self.write_reg(Register::MEAS_CFG, 0)
    }

    /// Read status bits from MEAS_CFG reg.
    /// MEAS_CFG register is masked with 0xF0
    pub fn read_status(&mut self) -> Result<u8, E> {
        let meas_cfg = self.read_reg(Register::MEAS_CFG)?;
        Ok(meas_cfg & 0xF0)
    }

    /// Returns the product ID from PROD_ID register.
    /// This value is expected to be 0x1D
    pub fn get_product_id(&mut self) -> Result<u8, E> {
        // TODO: Make sure that both revision ID and product ID is supported. Sec 8.10
        self.read_reg(Register::PROD_ID)
    }

    /// Start a single or continuous measurement for `pres`sure or `temp`erature
    pub fn trigger_measurement(
        &mut self,
        temp: bool,
        pres: bool,
        continuous: bool,
    ) -> Result<(), E> {
        if (!continuous && temp && pres) || (continuous && !temp && !pres) {
            // unsupported mode, See Sec 8.5 (MEAS_CFG), MEAS_CTRL field values
            // FIXME: Implement error return
        }
        // See section 8.5, MEAS_CTRL field description in manual

        let mut meas_cfg: u8 = self.read_reg(Register::MEAS_CFG)?;
        meas_cfg = (meas_cfg >> 3) << 3; // reset last 3 bits
        meas_cfg |= (continuous as u8) << 2 | (temp as u8) << 1 | pres as u8;

        self.write_reg(Register::MEAS_CFG, meas_cfg)
    }

    /// returns the sensor_ready bit from the status register
    pub fn init_complete(&mut self) -> Result<bool, E> {
        // see  sec 8.5, MEAS_CFG, SENSOR_RDY field (bit 6)
        let status = self.read_status()?;
        Ok((status & 0x64) != 0)
    }

    /// returns the temp_ready bit from the status register
    pub fn temp_ready(&mut self) -> Result<bool, E> {
        // See sec 8.5 TMP_RDY field
        let status = self.read_status()?;
        Ok((status & 0x20) != 0)
    }

    pub fn pres_ready(&mut self) -> Result<bool, E> {
        // See sec 8.5 PRS_RDY field
        let status = self.read_status()?;
        Ok((status & 0x10) != 0)
    }

    /// Read raw temperature contents
    fn read_temp_raw(&mut self) -> Result<i32, E> {
        let mut bytes: [u8; 3] = [0, 0, 0];
        self.i2c
            .write_read(self.address, &[Register::TMP_B2.addr()], &mut bytes)?;
        let temp = ((bytes[0] as u32) << 16) | ((bytes[1] as u32) << 8) | (bytes[2] as u32);
        let temp = self.get_twos_complement(temp, 24);

        Ok(temp)
    }

    /// See section 4.9.2:
    fn read_temp_scaled(&mut self) -> Result<f32, E> {
        let raw_sc: f32 = self.read_temp_raw()? as f32 / self.temp_res.get_kt_value();
        Ok(raw_sc)
    }

    /// Read calibrated temperature data in degrees Celsius
    /// This method uses the pre calculated constants based on the calibration coefficients
    /// See section 4.9.2 in the datasheet (formula), Sec 8.11 (coefficients)
    pub fn read_temp_calibrated(&mut self) -> Result<f32, E> {
        let scaled = self.read_temp_scaled();
        match scaled {
            Ok(raw_sc) => Ok((self.coeffs.C0 as f32 * 0.5) + (self.coeffs.C1 as f32 * raw_sc)),
            Err(err) => Err(err),
        }
    }

    /// Read raw pressure contents
    pub fn read_pressure_raw(&mut self) -> Result<i32, E> {
        let mut bytes: [u8; 3] = [0, 0, 0];
        self.i2c
            .write_read(self.address, &[Register::PSR_B2.addr()], &mut bytes)?;

        let pressure: i32 =
            (((bytes[0] as i32) << 24) | ((bytes[1] as i32) << 16) | (bytes[2] as i32) << 8) >> 8;

        Ok(pressure)
    }

    fn read_pressure_scaled(&mut self) -> Result<f32, E> {
        let pres_raw = self.read_pressure_raw()?;
        let k_p = self.pres_res.get_kP_value();
        let pres_scaled = pres_raw as f32 / k_p;

        Ok(pres_scaled)
    }

    /// Read calibrated pressure data in Pa
    /// This method uses the calibration coefficients
    /// See section 8.11 in the datasheet
    /// See section 4.9.1 for calculation method
    pub fn read_pressure_calibrated(&mut self) -> Result<f32, E> {
        let pres_scaled = self.read_pressure_scaled()?;
        let temp_scaled = self.read_temp_scaled()?;

        let pres_cal = (self.coeffs.C00 as f32)
            + (pres_scaled
                * (self.coeffs.C10 as f32
                    + pres_scaled
                        * (self.coeffs.C20 as f32 + pres_scaled * self.coeffs.C30 as f32)))
            + (temp_scaled * self.coeffs.C01 as f32)
            + (temp_scaled
                * pres_scaled
                * (self.coeffs.C11 as f32 + pres_scaled * self.coeffs.C21 as f32));

        Ok(pres_cal)
    }

    /// Issue a full reset and fifo flush
    pub fn reset(&mut self) -> Result<(), E> {
        self.write_reg(Register::RESET, 0b10001001)
    }

    fn write_reg(&mut self, reg: Register, value: u8) -> Result<(), E> {
        let bytes = [reg.addr(), value];
        self.i2c.write(self.address, &bytes)
    }

    fn read_reg(&mut self, reg: Register) -> Result<u8, E> {
        let mut buffer: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[reg.addr()], &mut buffer)?;
        Ok(buffer[0])
    }

    /// Read calibration coefficients. Taken from official Arduino library
    ///
    /// See https://github.com/Infineon/DPS310-Pressure-Sensor/blob/888200c7efd8edb19ce69a2144e28ba31cdad449/src/Dps310.cpp#L89
    /// See Sec 8.11
    fn read_calibration_coefficients(&mut self) -> Result<(), E> {
        let mut bytes: [u8; 18] = [0; 18];

        self.i2c
            .write_read(self.address, &[Register::COEFF_REG_1.addr()], &mut bytes)?;

        let _c0 = ((bytes[0] as u32) << 4) | (((bytes[1] as u32) >> 4) & 0x0F);

        // C0 12bits 2's complement
        self.coeffs.C0 = self.get_twos_complement(
            ((bytes[0] as u32) << 4) | (((bytes[1] as u32) >> 4) & 0x0F),
            12,
        );

        let _c1 = (((bytes[1] as u32) & 0x0F) << 8) | (bytes[2] as u32);
        // C1 12bits 2's complement
        self.coeffs.C1 =
            self.get_twos_complement((((bytes[1] as u32) & 0x0F) << 8) | (bytes[2] as u32), 12);

        let _c00 = ((bytes[3] as u32) << 12)
            | ((bytes[4] as u32) << 4)
            | (((bytes[5] as u32) >> 4) & 0x0F);

        // C00 20bits 2's complement
        self.coeffs.C00 = self.get_twos_complement(
            ((bytes[3] as u32) << 12)
                | ((bytes[4] as u32) << 4)
                | (((bytes[5] as u32) >> 4) & 0x0F),
            20,
        );

        let _c10 =
            (((bytes[5] as u32) & 0x0F) << 16) | ((bytes[6] as u32) << 8) | (bytes[7] as u32);
        // C10 20bits 2's complement
        self.coeffs.C10 = self.get_twos_complement(
            (((bytes[5] as u32) & 0x0F) << 16) | ((bytes[6] as u32) << 8) | (bytes[7] as u32),
            20,
        );

        let _c01 = ((bytes[8] as u32) << 8) | (bytes[9] as u32);

        // C01 16bits 2's complement
        self.coeffs.C01 =
            self.get_twos_complement(((bytes[8] as u32) << 8) | (bytes[9] as u32), 16);

        let _c11 = ((bytes[10] as u32) << 8) | (bytes[11] as u32);

        // C11 16bits 2's complement
        self.coeffs.C11 =
            self.get_twos_complement(((bytes[10] as u32) << 8) | (bytes[11] as u32), 16);

        let _c20 = ((bytes[12] as u32) << 8) | (bytes[13] as u32);

        // C20 16bits 2's complement
        self.coeffs.C20 =
            self.get_twos_complement(((bytes[12] as u32) << 8) | (bytes[13] as u32), 16);

        let _c21 = ((bytes[14] as u32) << 8) | (bytes[15] as u32);

        // C21 16bits 2's complement
        self.coeffs.C21 =
            self.get_twos_complement(((bytes[14] as u32) << 8) | (bytes[15] as u32), 16);

        let _c30 = ((bytes[16] as u32) << 8) | (bytes[17] as u32);

        // C30 16bits 2's complement
        self.coeffs.C30 =
            self.get_twos_complement(((bytes[16] as u32) << 8) | (bytes[17] as u32), 16);

        Ok(())
    }

    fn get_twos_complement(&self, val: u32, length: u8) -> i32 {
        let mut ret = val as i32;
        if (val & ((1 as u32) << (length - 1))) > 0 {
            ret -= ((1 as u32) << length) as i32;
        }

        ret
    }
}
