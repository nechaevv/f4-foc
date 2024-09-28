use stm32f4xx_hal::{nb, spi, ClearFlags};
use stm32f4xx_hal::spi::Error;
use crate::peripherals::OutputPin;

pub struct At5048aDaisyChain<SPI: spi::Instance, CS: OutputPin, const N: usize> {
    bus: spi::Spi<SPI, false, u16>,
    cs: CS,
    read_index: usize
}

pub struct DaisyChain<const N:usize>;

impl <const N:usize> DaisyChain<N> {
    pub fn init<SPI: spi::Instance, CS: OutputPin>(
        bus: spi::Spi<SPI, false, u16>,
        cs: CS) -> At5048aDaisyChain<SPI, CS, N> {
        At5048aDaisyChain { bus, cs, read_index: 0 }
    }
}


const READ_COMMAND: u16 = 0xFFFFu16;

impl<SPI: spi::Instance, CS: OutputPin, const N:usize> At5048aDaisyChain<SPI, CS, N> {

    pub fn read_start(&mut self) {
        self.cs.set_low();
        self.read_index = 0;
        self.bus.write_nonblocking(READ_COMMAND).unwrap();
    }

    pub fn on_spi_rxne(&mut self) -> Result<(i32, usize), Error> {
        self.bus.clear_all_flags();
        match nb::block!(self.bus.read_nonblocking()) {
            Err(e) => {
                self.cs.set_high();
                Err(e)
            }
            Ok(raw_data) =>
                {
                    let result = (((raw_data & 0x3FFF) as i32) - 0x1FFF) << 17;
                    let current_index = self.read_index;
                    self.read_index += 1;
                    if self.read_index >= N {
                        self.read_index = 0;
                        self.cs.set_high();
                    } else {
                        self.bus.write_nonblocking(READ_COMMAND).unwrap();
                    }
                    Ok((result, current_index))
                }
        }
    }
}