use crate::success::Success;
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::time::Hertz;

#[derive(Debug)]
pub enum Dht22Error {
    InitializationError,
    ChecksumError,
}

#[derive(Debug)]
pub struct Dht22Sensor<P> {
    initialization_ticks: u32,
    response_ticks: u32,
    microsecond_ticks: u32,
    pin: P,
}

impl<P> Dht22Sensor<P>
where
    P: InputPin + OutputPin,
{
    pub fn new(pin: P, sysclk: Hertz) -> Dht22Sensor<P> {
        assert!(sysclk.0 >= 1_000_000);

        Dht22Sensor {
            initialization_ticks: sysclk.0 / 50,
            response_ticks: sysclk.0 / 25_000,
            microsecond_ticks: sysclk.0 / 1_000_000,
            pin,
        }
    }

    pub fn read(&mut self) -> Result<(u16, i16), Dht22Error> {
        let _ = self.pin.set_low();
        cortex_m::asm::delay(self.initialization_ticks);
        let _ = self.pin.set_high();
        cortex_m::asm::delay(self.response_ticks);

        self.wait_high()?;
        self.wait_low()?;
        self.wait_high()?;

        let humidity_high = self.read_byte();
        let humidity_low = self.read_byte();
        let temperature_high = self.read_byte();
        let temperature_low = self.read_byte();
        let checksum = self.read_byte();

        if humidity_high
            .wrapping_add(humidity_low.wrapping_add(temperature_high.wrapping_add(temperature_low)))
            == checksum
        {
            let humidity = (humidity_high as u16) << 8 | humidity_low as u16;
            let temperature = match (temperature_high & 0x80 == 0, temperature_high & 0x7f) {
                (true, value) => (value as i16) << 8 | temperature_low as i16,
                (false, value) => -((value as i16) << 8 | temperature_low as i16),
            };

            Ok((humidity, temperature))
        } else {
            Err(Dht22Error::ChecksumError)
        }
    }

    #[inline]
    fn wait_high(&self) -> Result<(), Dht22Error> {
        let mut iterations = 0;

        while self.pin.is_high().success() {
            if iterations > self.initialization_ticks * 2 {
                return Err(Dht22Error::InitializationError);
            }

            iterations += 1;
        }

        Ok(())
    }

    #[inline]
    fn wait_low(&self) -> Result<(), Dht22Error> {
        let mut iterations = 0;

        while self.pin.is_low().success() {
            if iterations > self.initialization_ticks * 2 {
                return Err(Dht22Error::InitializationError);
            }

            iterations += 1;
        }

        Ok(())
    }

    #[inline]
    fn read_byte(&self) -> u8 {
        let mut result = 0;

        for _ in 0..8 {
            result = result << 1;

            if self.read_bit() {
                result |= 1;
            }
        }

        result
    }

    #[inline]
    fn read_bit(&self) -> bool {
        let mut low_loops = 0;
        let mut high_loops = 0;

        while self.pin.is_low().success() {
            low_loops += 1;

            cortex_m::asm::delay(self.microsecond_ticks);
        }

        while self.pin.is_high().success() {
            high_loops += 1;

            cortex_m::asm::delay(self.microsecond_ticks);
        }

        high_loops > low_loops
    }
}
