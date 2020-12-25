# DPS310 - embedded-hal I2C driver crate

![](https://img.shields.io/crates/v/dps310.svg)
![](https://docs.rs/dps310/badge.svg)

A platform agnostic driver to interface with the DPS310 barometric pressure & temp sensor.
This driver uses I2C via [embedded-hal](https://docs.rs/embedded-hal). Note that the DPS310 also supports SPI, however that is not (yet) implemented in this driver.

## Usage

Include this [crate](https://crates.io/crates/dps310) as a dependency in your Cargo.toml


```
[dependencies.dps310]
version = "<version>"
```

Use embedded-hal implementation to get I2C, then create a driver instance

```rust
use dps310::{DPS310, self};


let address = 0x77;
let mut dps = DPS310::new(i2c, address, &dps310::Config::new()).unwrap();

dps.trigger_measurement(true, true, true).unwrap();

if dps.data_ready().unwrap() {
    let pressure = dps.read_pressure_calibrated().unwrap();
    let temp = dps.read_temp_calibrated().unwrap();
    iprintln!(stim, "pressure: {:.1} [Pa]\t temp: {:.1} [˚C]", pressure, temp);
}
```