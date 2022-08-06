// Sample code for stm32f3discovery (STM32F303VC) board


// Add following to Cargo.toml
// m = "0.1.1"
// f3 = "0.6.1"
// cortex-m = "0.6.3"
// cortex-m-rt = "0.6.3"
// panic-itm = "0.4.0"
// panic-halt = "0.2.0"
// dps310 = "0.1.2"

#![no_main]
#![no_std]

use core::convert::TryInto;

pub use cortex_m::{asm::bkpt, iprint, iprintln, peripheral::ITM};
pub use cortex_m_rt::entry;
pub use stm32f3xx_hal::{
    delay::Delay, i2c::I2c, prelude::*, pac::Peripherals
};
use panic_halt as _;

use dps310::{DPS310, self};

const ADDRESS: u8 = 0x77;

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    // setup ITM output
    let  stim = &mut cp.ITM.stim[0];
    
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut delay = Delay::new(cp.SYST, clocks);

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);

    // Configure I2C1
    let mut scl =
        gpiob
            .pb6
            .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let mut sda =
        gpiob
            .pb7
            .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    scl.internal_pull_up(&mut gpiob.pupdr, true);
    sda.internal_pull_up(&mut gpiob.pupdr, true);
    let i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        100.kHz().try_into().unwrap(),
        clocks,
        &mut rcc.apb1,
    );

    let mut dps = DPS310::new(i2c, ADDRESS, &dps310::Config::new()).unwrap();
    let mut init_done = false;
    iprintln!(stim, "Wait for init done..");

    while !init_done {
        let compl = dps.init_complete();
        init_done = match compl {
            Ok(c) => c,
            Err(_e) => false
        };

        delay.delay_ms(200_u8);
    }

    iprintln!(stim, "pressure sensor init done");
    dps.trigger_measurement(true, true, true).unwrap();

    loop {
        delay.delay_ms(200_u8);
        if dps.temp_ready().unwrap() {
            let temp = dps.read_temp_calibrated().unwrap();
            iprintln!(stim, "Temperature: {:.1} [C]", temp);
        }

        if dps.pres_ready().unwrap() {
            let pressure = dps.read_pressure_calibrated().unwrap();
            iprintln!(stim, "pressure: {:.1} [Pa]", pressure);
            
        }
        
    }
}
