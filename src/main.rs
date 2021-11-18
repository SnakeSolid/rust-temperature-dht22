#![deny(unsafe_code)]
#![no_main]
#![no_std]

mod dht22;
mod success;

use crate::success::Success;
use core::fmt::Write;
use cortex_m_rt::entry;
use dht22::Dht22Sensor;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;
use stm32f1xx_hal::adc;
use stm32f1xx_hal::afio::AfioExt;
use stm32f1xx_hal::delay::Delay;
use stm32f1xx_hal::flash::FlashExt;
use stm32f1xx_hal::gpio::GpioExt;
use stm32f1xx_hal::pac;
use stm32f1xx_hal::rcc::RccExt;
use stm32f1xx_hal::serial::Config;
use stm32f1xx_hal::serial::Serial;
use stm32f1xx_hal::time::U32Ext;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut adc = adc::Adc::adc1(dp.ADC1, &mut rcc.apb2, clocks);
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let mut dht22 = gpioa.pa0.into_open_drain_output(&mut gpioa.crl);
    let bluetooth_tx = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    let bluetooth_rx = gpiob.pb11;
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    let mut delay = Delay::new(cp.SYST, clocks);

    dht22.set_high().success();
    led.set_high().success();

    let mut sensor = Dht22Sensor::new(dht22, clocks.sysclk());
    let (mut bluetooth_tx, _) = Serial::usart3(
        dp.USART3,
        (bluetooth_tx, bluetooth_rx),
        &mut afio.mapr,
        Config::default().baudrate(9_600.bps()),
        clocks,
        &mut rcc.apb1,
    )
    .split();

    writeln!(bluetooth_tx, "CPU;Humidity;Temperature").success();

    loop {
        let _ = led.set_low();
        let cpu_temp = adc.read_temp();

        match sensor.read() {
            Ok((humidity, temperature)) => {
                writeln!(
                    bluetooth_tx,
                    "{};{}.{};{}.{}",
                    cpu_temp,
                    humidity / 10,
                    humidity % 10,
                    temperature / 10,
                    temperature % 10,
                )
                .success();
            }
            Err(_) => {
                writeln!(bluetooth_tx, "{};;", cpu_temp).success();
            }
        };

        let _ = led.set_high();

        delay.delay_ms(60_000u16);
    }
}
