// Sample code for stm32f3discovery (STM32F303VC) board


// Add following to Cargo.toml
// m = "0.1.1"
// f3 = "0.6.1"
// cortex-m = "0.6.3"
// cortex-m-rt = "0.6.3"
// panic-itm = "0.4.0"
// panic-halt = "0.2.0"
// dps310 = "0.1.0"

#![feature(core_intrinsics)]
#![no_main]
#![no_std]
#[allow(unused_imports)]
#[allow(unused_extern_crates)] // NOTE(allow) bug rust-lang/rust#53964


extern crate panic_itm; // panic handler

pub use cortex_m::{asm::bkpt, iprint, iprintln, peripheral::ITM};
pub use cortex_m_rt::entry;
pub use f3::{
    hal::{delay::Delay, i2c::I2c, prelude::*, stm32f30x},
};


use dps310::{DPS310, self};


#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32f30x::Peripherals::take().unwrap();

    // setup ITM output
    let  stim = &mut cp.ITM.stim[0];
    
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut delay = Delay::new(cp.SYST, clocks);

    
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let scl = gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
    let sda = gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl);

    let i2c = I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks, &mut rcc.apb1);
    
    
    let address = 0x77;

    let mut dps = DPS310::new(i2c, address, &dps310::Config::new()).unwrap();
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
